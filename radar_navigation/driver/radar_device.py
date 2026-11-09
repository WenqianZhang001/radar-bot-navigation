"""Talks to a TI AWR1843BOOST running the mmWave SDK 3.x out-of-box demo. No ROS code here.

The board shows up as two serial ports through its XDS110 USB chip:
  - command port (interface 00, 115200 baud): takes the .cfg lines, answers "Done"
  - data port    (interface 03, 921600 baud): streams one binary packet per radar frame
"""
import glob
import struct
import time

import serial

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'   # start of every frame packet
HEADER_LEN = 40                               # magic + 8 x uint32
TLV_POINTS = 1                                # per point: x, y, z, velocity (float32)
TLV_SIDE_INFO = 7                             # per point: snr, noise (int16, units of 0.1 dB)
PROMPT = b':/>'                               # end of the CLI prompt "mmwDemo:/>"


def find_ports():
    """Return (command_port, data_port) of the radar's USB interface; None where not found."""
    cli = sorted(glob.glob('/dev/serial/by-id/*XDS110*-if00*'))
    data = sorted(glob.glob('/dev/serial/by-id/*XDS110*-if03*'))
    if len(cli) > 1 or len(data) > 1:
        raise RuntimeError('multiple XDS110 radars found; supply explicit cli_port and data_port')
    return (cli[0] if cli else None, data[0] if data else None)


class RadarCli:
    """The radar's command port."""

    def __init__(self, port):
        # exclusive: refuse to share the radar with another program (e.g. radar_viewer.py)
        self.serial = serial.Serial(port, 115200, timeout=0.1, exclusive=True)

    def command(self, cmd, timeout=3.0):
        """Type one command and return the radar's reply, up to its next prompt."""
        # The radar drops characters when a whole line arrives at once,
        # so type the command one character at a time.
        for ch in (cmd + '\n').encode():
            self.serial.write(bytes([ch]))
            time.sleep(0.001)
        reply, deadline = b'', time.time() + timeout
        while PROMPT not in reply and time.time() < deadline:
            reply += self.serial.read(self.serial.in_waiting or 1)
        return reply.decode(errors='ignore')

    def configure(self, cfg_path):
        """Send every line of a .cfg file. Returns [(line, reply)] for the lines that failed."""
        with open(cfg_path) as f:
            lines = [l.strip() for l in f if l.strip() and not l.lstrip().startswith('%')]
        # An empty line clears anything half-typed in the CLI and checks that it answers.
        self.serial.reset_input_buffer()
        if PROMPT.decode() not in self.command('', timeout=1.0):
            raise RuntimeError('radar is not answering on its command port; '
                               'press the reset button on the radar board and try again')
        failed = []
        for line in lines:
            reply = self.command(line)
            if 'Done' not in reply and line not in ('sensorStop', 'flushCfg'):
                failed.append((line, reply.replace('mmwDemo:/>', '').strip()))
        return failed

    def close(self):
        self.serial.close()


class FrameParser:
    """Turns the data port's byte stream into frames. Feed it whatever bytes arrive."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        """Add bytes; return the frames completed so far as a list of (frame_number, points).

        Each point is (x, y, z, velocity, snr_db, noise_db) in the radar's own TI axes:
        x to the right, y straight out of the antennas, z up; velocity is radial,
        positive when moving away from the radar.
        """
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(MAGIC)
            if start < 0:
                del self.buf[:-7]            # keep a magic word that may be split across reads
                return frames
            del self.buf[:start]
            if len(self.buf) < HEADER_LEN:
                return frames
            total_len, frame_num, num_tlv = (struct.unpack_from('<8I', self.buf, 8)[i] for i in (1, 3, 6))
            if total_len < HEADER_LEN:
                del self.buf[:8]             # broken header: skip it and look for the next frame
                continue
            if len(self.buf) < total_len:
                if self.buf.find(MAGIC, 8) >= 0:
                    del self.buf[:8]         # next frame began before this one finished: drop it
                    continue
                return frames
            frames.append((frame_num, self._points(bytes(self.buf[:total_len]), num_tlv)))
            del self.buf[:total_len]

    @staticmethod
    def _points(frame, num_tlv):
        xyzv, side = [], []
        offset = HEADER_LEN
        for _ in range(num_tlv):
            if offset + 8 > len(frame):
                break
            tlv_type, tlv_len = struct.unpack_from('<2I', frame, offset)
            body = frame[offset + 8:offset + 8 + tlv_len]
            if tlv_type == TLV_POINTS:
                xyzv = list(struct.iter_unpack('<4f', body[:len(body) // 16 * 16]))
            elif tlv_type == TLV_SIDE_INFO:
                side = [(snr / 10.0, noise / 10.0)
                        for snr, noise in struct.iter_unpack('<2h', body[:len(body) // 4 * 4])]
            offset += 8 + tlv_len
        if len(side) != len(xyzv):
            side = [(float('nan'), float('nan'))] * len(xyzv)
        return [p + s for p, s in zip(xyzv, side)]
