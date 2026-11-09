"""TI DCA1000EVM raw-ADC capture over Ethernet, for an xWR18xx running the mmWave SDK 3.x demo.

Network (DCA1000 factory defaults):
    DCA1000  192.168.33.180   takes commands on UDP port 4096
    Jetson   192.168.33.30    gets command replies on UDP 4096 and raw ADC data on UDP 4098

Commands and replies are little-endian: 0xA55A, command code, payload length, payload, 0xEEAA.
Data packets are: sequence number (uint32), byte count (uint48), raw ADC bytes.

    python3 dca1000.py info     check that the DCA1000 answers and print its FPGA version
    python3 dca1000.py stop     stop a recording left running by an interrupted capture
"""
import socket
import struct
import sys
import time
from collections import namedtuple

import numpy as np

DCA_IP, HOST_IP = '192.168.33.180', '192.168.33.30'
CONFIG_PORT, DATA_PORT = 4096, 4098

_HEADER, _FOOTER = 0xA55A, 0xEEAA
RESET_FPGA = 0x01
CONFIG_FPGA_GEN = 0x03
RECORD_START = 0x05
RECORD_STOP = 0x06
SYSTEM_CONNECT = 0x09
ASYNC_STATUS = 0x0A          # sent unprompted by the DCA1000 when something happens
CONFIG_PACKET_DATA = 0x0B
READ_FPGA_VERSION = 0x0E
_NAMES = {RESET_FPGA: 'RESET_FPGA', CONFIG_FPGA_GEN: 'CONFIG_FPGA_GEN',
          RECORD_START: 'RECORD_START', RECORD_STOP: 'RECORD_STOP',
          SYSTEM_CONNECT: 'SYSTEM_CONNECT', CONFIG_PACKET_DATA: 'CONFIG_PACKET_DATA',
          READ_FPGA_VERSION: 'READ_FPGA_VERSION'}
# Codes carried by ASYNC_STATUS messages (names as in TI's DCA1000 CLI source).
ASYNC_NAMES = {0: 'no LVDS data', 1: 'no header', 2: 'EEPROM failure', 3: 'SD card detected',
               4: 'SD card removed', 5: 'SD card full', 6: 'mode config failure', 7: 'DDR full',
               8: 'record completed', 9: 'LVDS buffer full', 10: 'playback completed',
               11: 'playback out of sequence'}

Frame = namedtuple('Frame', 'index data lost_bytes')


class DCA1000:
    """Commands and data stream of one DCA1000EVM."""

    def __init__(self, host_ip=HOST_IP, dca_ip=DCA_IP, rcvbuf=32 << 20):
        self.dest = (dca_ip, CONFIG_PORT)
        self.ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.ctrl.bind((host_ip, CONFIG_PORT))
            # Each radar frame arrives as a burst far faster than the average data rate;
            # a large socket buffer absorbs it. The kernel caps this at net.core.rmem_max.
            self.data.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
            self.data.bind((host_ip, DATA_PORT))
        except OSError as e:
            self.close()
            raise OSError(f'cannot open UDP ports {CONFIG_PORT}/{DATA_PORT} on {host_ip}: {e}. '
                          f'Is the wired port set to {host_ip}/24, and is no other capture running?') from e
        self.rcvbuf = self.data.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.events = []                  # (code, name) of every ASYNC_STATUS message seen
        self.packets = self.lost_packets = 0
        self.byte_count_mode = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.ctrl.close()
        self.data.close()

    # ---- commands -------------------------------------------------------------------------

    def command(self, code, payload=b'', timeout=2.0):
        """Send one command and return the status word of its reply (0 means success)."""
        self.ctrl.sendto(struct.pack('<3H', _HEADER, code, len(payload)) + payload
                         + struct.pack('<H', _FOOTER), self.dest)
        deadline = time.monotonic() + timeout
        while True:
            self.ctrl.settimeout(max(deadline - time.monotonic(), 0.001))
            try:
                reply = self.ctrl.recv(2048)
            except socket.timeout:
                raise TimeoutError(f'DCA1000 did not answer {_NAMES.get(code, hex(code))}; '
                                   'check its power and Ethernet cable') from None
            if len(reply) < 8:
                continue
            header, reply_code, status = struct.unpack_from('<3H', reply)
            if header != _HEADER:
                continue
            if reply_code == ASYNC_STATUS:
                self.events.append((status, ASYNC_NAMES.get(status, 'unknown')))
            elif reply_code == code:
                return status

    def _require(self, code, payload=b'', hint=''):
        status = self.command(code, payload)
        if status != 0:
            raise RuntimeError(f'DCA1000 rejected {_NAMES[code]} (status {status}){hint}')

    def connect(self):
        self._require(SYSTEM_CONNECT)

    def fpga_version(self):
        """Return (major, minor, playback_mode)."""
        v = self.command(READ_FPGA_VERSION)
        return v & 0x7F, (v >> 7) & 0x7F, bool(v & 0x4000)

    def configure_fpga(self, lvds_lanes=2, lvds_timeout_s=30):
        """Raw logging mode, LVDS capture, Ethernet streaming, 16-bit samples.

        xWR16xx/18xx/68xx radars use 2 LVDS lanes; xWR12xx/14xx use 4.
        """
        payload = bytes([1, {4: 1, 2: 2}[lvds_lanes], 1, 2, 3, lvds_timeout_s])
        self._require(CONFIG_FPGA_GEN, payload,
                      hint='; check that DIP switch SW2 position 5 is on SW_CONFIG')

    def configure_packets(self, size=1472, delay_us=25):
        """UDP packet size in bytes and gap between packets (counted in 8 ns FPGA ticks)."""
        self._require(CONFIG_PACKET_DATA, struct.pack('<3H', size, round(delay_us * 125), 0))

    def start_record(self):
        self._require(RECORD_START)

    def stop_record(self):
        """Stop recording. Returns the status, or None if the DCA1000 did not answer."""
        try:
            return self.command(RECORD_STOP)
        except TimeoutError:
            return None

    # ---- data -----------------------------------------------------------------------------

    def drain(self):
        """Throw away data packets still queued from an earlier run; return how many."""
        self.data.setblocking(False)
        n = 0
        try:
            while True:
                self.data.recv(2048)
                n += 1
        except BlockingIOError:
            pass
        finally:
            self.data.setblocking(True)
        return n

    def frames(self, frame_bytes, timeout=2.0, first_timeout=10.0):
        """Yield Frame(index, data, lost_bytes) for each radar frame, in order.

        Each packet is placed by its byte count, so a lost packet leaves zeros in its frame
        (reported in lost_bytes) instead of shifting all later data.
        Raises TimeoutError when no packet arrives for `timeout` s (`first_timeout` at the start).
        """
        pkt = bytearray(2048)
        view = memoryview(pkt)
        frame, got, index, last_seq = bytearray(frame_bytes), 0, None, None
        self.packets = self.lost_packets = 0
        self.data.settimeout(first_timeout)
        while True:
            try:
                n = self.data.recv_into(pkt)
            except socket.timeout:
                raise TimeoutError('no data from the DCA1000 for '
                                   f'{first_timeout if index is None else timeout} s') from None
            self.data.settimeout(timeout)
            if n <= 10:
                continue
            seq = struct.unpack_from('<I', pkt)[0]
            count = int.from_bytes(pkt[4:10], 'little')
            size = n - 10
            if self.byte_count_mode is None:   # is the byte count taken before or after this packet?
                self.byte_count_mode = 'after' if count == size else 'before'
            pos = count - size if self.byte_count_mode == 'after' else count
            self.packets += 1
            if last_seq is not None and seq > last_seq + 1:
                self.lost_packets += seq - last_seq - 1
            last_seq = seq
            if index is None:
                index = pos // frame_bytes
            done = 0
            while done < size:
                f, at = divmod(pos + done, frame_bytes)
                take = min(size - done, frame_bytes - at)
                while f > index:              # the stream skipped past the end of this frame
                    yield Frame(index, frame, frame_bytes - got)
                    frame, got, index = bytearray(frame_bytes), 0, index + 1
                if f == index:
                    frame[at:at + take] = view[10 + done:10 + done + take]
                    got += take
                    if got == frame_bytes:
                        yield Frame(index, frame, 0)
                        frame, got, index = bytearray(frame_bytes), 0, index + 1
                done += take


# ---- radar side ------------------------------------------------------------------------------

def to_iq(raw, n_chirps, n_rx, n_samples, q_first=True):
    """Convert one frame of DCA1000 bytes into complex64 samples shaped (chirp, rx, sample).

    xWR16xx/18xx/68xx send complex 16-bit samples over 2 LVDS lanes, and the DCA1000 stores
    every pair of samples as four int16 words: both of one component, then both of the other.
    With the SDK demo's adcbufCfg SampleSwap=1 the order is [Q0 Q1 I0 I1] (q_first=True);
    with SampleSwap=0 it is [I0 I1 Q0 Q1]. The SampleSwap=1 order was checked on the AWR1843BOOST
    against the radar's own range profile; TI's MATLAB reader assumes [I0 I1 Q0 Q1], which would
    mirror the spectrum for SampleSwap=1 data.
    """
    w = np.frombuffer(raw, dtype='<i2').reshape(-1, 4)
    first, second = w[:, 0:2].reshape(-1), w[:, 2:4].reshape(-1)
    i, q = (second, first) if q_first else (first, second)
    iq = np.empty(i.size, np.complex64)
    iq.real, iq.imag = i, q
    return iq.reshape(n_chirps, n_rx, n_samples)


class RadarConfig:
    """What the raw stream looks like for a given mmWave SDK 3.x .cfg file."""

    def __init__(self, path):
        self.path = path
        with open(path) as f:
            self.lines = [l.strip() for l in f if l.strip() and not l.lstrip().startswith('%')]
        cmds = {}
        for line in self.lines:
            name, *vals = line.split()
            cmds.setdefault(name, []).append(vals)

        def one(name):
            if len(cmds.get(name, [])) != 1:
                raise ValueError(f'{path}: expected exactly one {name} line')
            return cmds[name][0]

        if 'advFrameCfg' in cmds or one('dfeDataOutputMode')[0] != '1':
            raise ValueError(f'{path}: only dfeDataOutputMode 1 (plain frames) is supported')
        rx_mask = int(one('channelCfg')[0])
        self.n_rx = bin(rx_mask).count('1')
        bits, out_fmt = (int(v) for v in one('adcCfg')[:2])
        if bits != 2 or out_fmt not in (1, 2):
            raise ValueError(f'{path}: needs 16-bit complex ADC output (adcCfg 2 1)')
        _, buf_fmt, swap, interleave, _ = (int(v) for v in one('adcbufCfg')[:5])
        if buf_fmt != 0 or interleave != 1:
            raise ValueError(f'{path}: needs complex, non-interleaved ADC buffer (adcbufCfg -1 0 1 1 1)')
        self.q_first = swap == 1
        p = one('profileCfg')
        self.start_ghz, self.idle_us, self.adc_start_us, self.ramp_end_us = (float(v) for v in p[1:5])
        self.slope_mhz_us = float(p[7])
        self.n_samples, self.sample_rate_ksps = int(p[9]), float(p[10])
        first, last, self.n_loops, self.n_frames = (int(v) for v in one('frameCfg')[:4])
        self.frame_period_ms = float(one('frameCfg')[4])
        self.chirps_per_loop = last - first + 1
        self.n_chirps = self.chirps_per_loop * self.n_loops
        tx_of = {}
        for c in cmds.get('chirpCfg', []):
            for i in range(int(c[0]), int(c[1]) + 1):
                tx_of[i] = int(c[7])
        # TX antenna(s) of each chirp in a loop, e.g. [1, 4, 2] = TX0, TX2, TX1
        self.chirp_tx_masks = [tx_of.get(i) for i in range(first, last + 1)]
        _, header, data_fmt, sw = (int(v) for v in one('lvdsStreamCfg')[:4])
        # The DCA1000 raw mode needs plain ADC data with no headers and no software session.
        self.raw_lvds = (data_fmt, header, sw) == (1, 0, 0)

    @property
    def frame_bytes(self):
        return self.n_chirps * self.n_rx * self.n_samples * 4

    @property
    def range_bin_m(self):
        """Range covered by one bin of an n_samples-point FFT (complex sampling)."""
        return 3e8 * self.sample_rate_ksps * 1e3 / (2 * self.slope_mhz_us * 1e12 * self.n_samples)

    def summary(self):
        return (f'{self.n_chirps} chirps x {self.n_rx} RX x {self.n_samples} samples per frame '
                f'({self.n_loops} loops of {self.chirps_per_loop} chirps), '
                f'{self.frame_bytes} bytes/frame, one frame every {self.frame_period_ms:g} ms, '
                f'{self.range_bin_m * 100:.2f} cm per range bin, '
                f'max range {self.range_bin_m * self.n_samples:.2f} m')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'info'
    if cmd not in ('info', 'stop'):
        sys.exit('usage: dca1000.py [info|stop]')
    with DCA1000() as dca:
        if cmd == 'info':
            dca.connect()
            major, minor, playback = dca.fpga_version()
            print(f'DCA1000 at {DCA_IP} answered: FPGA version {major}.{minor}, '
                  f'{"playback" if playback else "record"} mode; '
                  f'socket receive buffer {dca.rcvbuf >> 20} MB')
        else:
            print('RECORD_STOP status:', dca.stop_record())
        for code, name in dca.events:
            print(f'DCA1000 async status {code}: {name}')
