#!/usr/bin/env python3
"""Record raw ADC (IQ) frames from the AWR1843BOOST through the DCA1000EVM.

    python3 capture_raw.py                          # 20 frames with awr1843_raw.cfg
    python3 capture_raw.py --frames 300 --cfg my_profile.cfg

The radar is configured over its USB command port and the DCA1000 over Ethernet. The DCA1000
is armed before the radar starts, so the stream begins exactly at frame 0, chirp 0.

Files written to --out (default ./captures), all named after the start time:
    *_raw.bin    the DCA1000 byte stream, frame after frame (the format TI's tools record)
    *_iq.npy     complex64, shape (frames, chirps, rx, samples). Chirps run loop after loop,
                 with the .cfg's TX order inside each loop (TX0, TX2, TX1 for awr1843_raw.cfg),
                 so iq.reshape(frames, loops, chirps_per_loop, rx, samples) separates the TXs
    *_meta.json  radar settings, packet statistics and lost bytes per frame
    *_uart.bin   (with --uart-log) the radar's own processed output from its data port
"""
import argparse
import glob
import json
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np
import serial

from dca1000 import DCA1000, RadarConfig, to_iq

HERE = os.path.dirname(os.path.abspath(__file__))


class RadarCli:
    """The radar's command port (XDS110 interface 00, 115200 baud)."""
    PROMPT = b':/>'

    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.1, exclusive=True)

    def command(self, line, timeout=3.0):
        # The radar drops characters when a whole line arrives at once, so type it one by one.
        for ch in (line + '\n').encode():
            self.serial.write(bytes([ch]))
            time.sleep(0.001)
        reply, deadline = b'', time.monotonic() + timeout
        while self.PROMPT not in reply and time.monotonic() < deadline:
            reply += self.serial.read(self.serial.in_waiting or 1)
        return reply.decode(errors='replace')

    def configure(self, lines):
        self.serial.reset_input_buffer()
        if self.PROMPT.decode() not in self.command('', timeout=1.0):
            raise RuntimeError('radar is not answering on its command port; press its NRST button')
        for line in lines:
            reply = self.command(line)
            if 'Done' not in reply and line.split()[0] not in ('sensorStop', 'flushCfg'):
                raise RuntimeError(f'radar rejected "{line}": {reply.strip()}')

    def close(self):
        self.serial.close()


def xds110_port(interface):
    ports = sorted(glob.glob(f'/dev/serial/by-id/*XDS110*-if0{interface}'))
    if len(ports) != 1:
        sys.exit('radar USB port not found (or more than one radar): '
                 'is the radar USB cable plugged into the Jetson?')
    return ports[0]


def log_uart(port, path, stop):
    with serial.Serial(port, 921600, timeout=0.1, exclusive=True) as s, open(path, 'wb') as f:
        s.reset_input_buffer()
        while not stop.is_set():
            f.write(s.read(4096))


def record(cfg, n_frames, raw_file, uart_path=None):
    """Configure both boards, record n_frames into raw_file, stop both. Returns a stats dict."""
    lost, arrival, stop, uart = [], [], threading.Event(), None
    cli = RadarCli(xds110_port(0))
    try:
        with DCA1000() as dca:
            if dca.rcvbuf < 8 << 20:
                print(f'warning: UDP receive buffer is only {dca.rcvbuf >> 10} KB, so packets may be '
                      'lost; raise it with: sudo sysctl -w net.core.rmem_max=33554432')
            dca.connect()
            major, minor, _ = dca.fpga_version()
            dca.stop_record()                     # in case an earlier capture was interrupted
            dca.configure_fpga(lvds_lanes=2)
            dca.configure_packets()
            print(f'DCA1000 (FPGA {major}.{minor}) configured; configuring radar ...')
            cli.configure([l for l in cfg.lines if l.split()[0] != 'sensorStart'])
            if uart_path:
                uart = threading.Thread(target=log_uart, args=(xds110_port(3), uart_path, stop))
                uart.start()
            dca.drain()
            dca.start_record()
            try:
                reply = cli.command('sensorStart', timeout=10.0)
                if 'Done' not in reply:
                    raise RuntimeError(f'radar rejected sensorStart: {reply.strip()}')
                print('recording ...')
                for frame in dca.frames(cfg.frame_bytes):
                    raw_file.write(frame.data)
                    lost.append(frame.lost_bytes)
                    arrival.append(time.monotonic())
                    if frame.lost_bytes:
                        print(f'  frame {frame.index}: {frame.lost_bytes} bytes lost')
                    if len(lost) % 10 == 0 or len(lost) == n_frames:
                        print(f'  {len(lost)}/{n_frames} frames')
                    if len(lost) == n_frames:
                        break
            except KeyboardInterrupt:
                print(f'interrupted: keeping the {len(lost)} frames recorded so far')
            finally:
                cli.command('sensorStop')
                dca.stop_record()
            return dict(fpga_version=f'{major}.{minor}', packets=dca.packets,
                        lost_packets=dca.lost_packets, lost_bytes_per_frame=lost,
                        frame_arrival_s=[round(t - arrival[0], 4) for t in arrival],
                        byte_count_mode=dca.byte_count_mode, socket_rcvbuf=dca.rcvbuf,
                        dca1000_async_events=dca.events)
    finally:
        stop.set()
        if uart:
            uart.join()
        cli.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', default=os.path.join(HERE, 'awr1843_raw.cfg'),
                    help='radar profile; must contain "lvdsStreamCfg -1 0 1 0"')
    ap.add_argument('--frames', type=int, default=20, help='frames to record (default 20)')
    ap.add_argument('--out', default=os.path.join(HERE, 'captures'), help='output folder')
    ap.add_argument('--uart-log', action='store_true',
                    help="also save the radar's processed output from its data port")
    args = ap.parse_args()

    cfg = RadarConfig(args.cfg)
    if not cfg.raw_lvds:
        sys.exit(f'{args.cfg}: set "lvdsStreamCfg -1 0 1 0" to stream raw ADC data to the DCA1000')
    print('radar profile:', cfg.summary())
    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.now()
    base = os.path.join(args.out, stamp.strftime('%Y%m%d_%H%M%S'))

    try:
        with open(base + '_raw.bin', 'wb') as raw_file:
            stats = record(cfg, args.frames, raw_file, base + '_uart.bin' if args.uart_log else None)
    except TimeoutError as e:
        sys.exit(f'{e}. The DCA1000 got no radar data: check the 60-pin ribbon cable, '
                 'that the radar profile has "lvdsStreamCfg -1 0 1 0", and the DCA1000 DIP switches.')
    n = len(stats['lost_bytes_per_frame'])
    if n == 0:
        sys.exit('no frames recorded')

    # Raw bytes -> complex IQ, one frame at a time so long captures fit in memory.
    raw = np.memmap(base + '_raw.bin', dtype=np.uint8, mode='r')
    iq = np.lib.format.open_memmap(base + '_iq.npy', mode='w+', dtype=np.complex64,
                                   shape=(n, cfg.n_chirps, cfg.n_rx, cfg.n_samples))
    for k in range(n):
        iq[k] = to_iq(raw[k * cfg.frame_bytes:(k + 1) * cfg.frame_bytes],
                      cfg.n_chirps, cfg.n_rx, cfg.n_samples, cfg.q_first)
    iq.flush()

    meta = dict(captured_at=stamp.isoformat(timespec='seconds'), cfg_file=os.path.abspath(args.cfg),
                cfg_lines=cfg.lines, iq_shape=list(iq.shape), iq_axes=['frame', 'chirp', 'rx', 'sample'],
                n_loops=cfg.n_loops, chirps_per_loop=cfg.chirps_per_loop,
                chirp_tx_masks=cfg.chirp_tx_masks, start_ghz=cfg.start_ghz,
                slope_mhz_us=cfg.slope_mhz_us, sample_rate_ksps=cfg.sample_rate_ksps,
                idle_us=cfg.idle_us, adc_start_us=cfg.adc_start_us, ramp_end_us=cfg.ramp_end_us,
                frame_period_ms=cfg.frame_period_ms, range_bin_m=cfg.range_bin_m,
                raw_word_order='Q0 Q1 I0 I1' if cfg.q_first else 'I0 I1 Q0 Q1', **stats)
    with open(base + '_meta.json', 'w') as f:
        json.dump(meta, f, indent=1)

    # Summary
    t = stats['frame_arrival_s']
    fps = (n - 1) / t[-1] if n > 1 and t[-1] > 0 else float('nan')
    lost_frames = sum(1 for b in stats['lost_bytes_per_frame'] if b)
    print(f'\n{n} frames ({fps:.1f} frames/s), {stats["packets"]} packets, '
          f'{stats["lost_packets"]} packets lost, {lost_frames} frames with missing data')
    sample = iq[:min(n, 50)]
    peak = max(np.abs(sample.real).max(), np.abs(sample.imag).max())
    print(f'largest ADC value {peak:.0f} of 32767' + ('  (saturating!)' if peak >= 32767 else ''))
    profile = (np.abs(np.fft.fft(sample * np.hanning(cfg.n_samples), axis=-1)) ** 2).mean(axis=(0, 1, 2))
    db = 10 * np.log10(profile + 1e-12)
    bins = np.argsort(profile[3:])[::-1][:3] + 3
    print('strongest returns: ' + ', '.join(f'{b * cfg.range_bin_m:.2f} m ({db[b] - np.median(db):.0f} dB '
                                             'above the median)' for b in bins))
    if stats['dca1000_async_events']:
        print('DCA1000 status messages:', stats['dca1000_async_events'])
    print(f'\nsaved {base}_iq.npy  shape {tuple(iq.shape)} (frame, chirp, rx, sample)')
    print(f'      {base}_raw.bin, {base}_meta.json' + (f', {base}_uart.bin' if args.uart_log else ''))


if __name__ == '__main__':
    main()
