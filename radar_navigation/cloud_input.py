"""Convert ROS radar clouds to the core's TI-axis Frame without ROS imports."""
import math
import struct
from .protocol import Frame


def decode_cloud(msg, sensor, expected_frame, sequence):
    if msg.header.frame_id != expected_frame:
        raise ValueError('Unexpected radar frame: '+msg.header.frame_id)
    fields = {f.name: f for f in msg.fields}
    velocity = 'velocity' if 'velocity' in fields else 'doppler'
    names = ['x', 'y', 'z', velocity]
    quality = all(n in fields for n in ('snr', 'noise'))
    if quality:
        names += ['snr', 'noise']
    for name in names:
        f = fields.get(name)
        if f is None or f.datatype != 7 or f.count != 1 or not 0 <= f.offset <= msg.point_step-4:
            raise ValueError('Expected scalar FLOAT32 field: '+name)
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('Truncated or invalid cloud layout')
    unpack = struct.Struct('>f' if msg.is_bigendian else '<f').unpack_from
    points, side = [], []
    for row in range(msg.height):
        for col in range(msg.width):
            base = row*msg.row_step+col*msg.point_step
            values = [unpack(msg.data, base+fields[n].offset)[0] for n in names]
            x, y, z, v = values[:4]
            if not all(math.isfinite(a) for a in (x, y, z, v)):
                continue
            points.append((-y, x, z, v))  # Core converts TI -> ROS exactly once.
            side.append(values[4:] if quality else [math.nan, math.nan])
    stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    return Frame(sensor, stamp, sequence, points, {
        'side_info_db': side, 'number_source': 'subscriber_sequence',
        'hardware_frame_number_available': False, 'source_frame': expected_frame,
        'source_velocity_field': velocity})
