"""Bounded streaming parser for TI mmWave SDK 3.x detected-point packets."""
import base64
import struct
import time
from dataclasses import asdict, dataclass, field

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'


@dataclass
class Frame:
    sensor: str
    stamp_ns: int
    number: int
    points: list
    metadata: dict = field(default_factory=dict)

    def to_dict(self):
        return asdict(self)


class Parser:
    def __init__(self, sensor="radar", max_packet=100000):
        self.sensor = sensor
        self.buffer = bytearray()
        self.max_packet = max_packet
        self.rejected = 0

    def feed(self, chunk, stamp_ns=None):
        self.buffer.extend(chunk)
        frames = []
        while True:
            index = self.buffer.find(MAGIC)
            if index < 0:
                self.buffer[:] = self.buffer[-7:]
                break
            del self.buffer[:index]
            if len(self.buffer) < 40:
                break
            version, size, platform, number, cycles, count, ntlv, sub = struct.unpack_from('<8I', self.buffer, 8)
            if not 40 <= size <= self.max_packet or ntlv > (size - 40) // 8:
                self.rejected += 1
                del self.buffer[:8]
                continue
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            del self.buffer[:size]
            points, side, unknown = [], [], []
            offset = 40
            try:
                for _ in range(ntlv):
                    kind, length = struct.unpack_from('<2I', packet, offset)
                    offset += 8
                    if offset + length > size:
                        raise ValueError("TLV exceeds packet")
                    payload = packet[offset:offset + length]
                    offset += length
                    if kind == 1:
                        if length % 16:
                            raise ValueError("Partial point")
                        points.extend(struct.iter_unpack('<4f', payload))
                    elif kind == 7:
                        if length % 4:
                            raise ValueError("Partial side information")
                        side.extend((snr * .1, noise * .1) for snr, noise in struct.iter_unpack('<2H', payload))
                    else:
                        unknown.append({"type": kind, "payload_b64": base64.b64encode(payload).decode()})
                if len(points) != count or (side and len(side) != count):
                    raise ValueError("Point count mismatch")
            except (ValueError, struct.error):
                self.rejected += 1
                continue
            frames.append(Frame(self.sensor, time.time_ns() if stamp_ns is None else stamp_ns, number,
                                [list(p) for p in points],
                                dict(version=version, platform=platform, cycles=cycles,
                                     subframe=sub, side_info_db=side, unknown_tlvs=unknown,
                                     timestamp_source="host_receipt")))
        return frames
