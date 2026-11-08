import math
import struct
import unittest
from types import SimpleNamespace as NS
from radar_navigation.cloud_input import decode_cloud
from radar_navigation.core import Filter, Pose, Grid


def cloud(rows, big=False, padding=0):
    names = ('x', 'y', 'z', 'velocity', 'snr', 'noise')
    fields = [NS(name=n, offset=i*4, datatype=7, count=1) for i, n in enumerate(names)]
    data = b''.join(struct.pack('>6f' if big else '<6f', *r)+b'\0'*padding for r in rows)
    return NS(header=NS(frame_id='radar_link', stamp=NS(sec=10, nanosec=12)),
              fields=fields, width=1, height=len(rows), point_step=24,
              row_step=24+padding, data=data, is_bigendian=big)


class CloudInputTests(unittest.TestCase):
    def test_geometry_quality_and_occupancy(self):
        msg = cloud([(2, .3, .2, .4, 15, 3)], True, 8)
        frame = decode_cloud(msg, 'front', 'radar_link', 7)
        self.assertEqual(frame.stamp_ns, 10000000012)
        self.assertFalse(frame.metadata['hardware_frame_number_available'])
        points = Filter(min_snr=10).update(frame, Pose(.036, -.065, 0, .185))
        for actual, expected in zip(points[0], (2.036, .235, .385, .4)):
            self.assertAlmostEqual(actual, expected, places=6)
        grid = Grid()
        grid.update(points, stamp=10)
        self.assertTrue(any(v >= 65 for v in grid.values()))
        grid.expire(12, 1.5)
        self.assertTrue(all(v == -1 for v in grid.values()))

    def test_unknown_quality_and_invalid_points(self):
        msg = cloud([(2, 0, .2, 0, math.nan, math.nan),
                     (math.nan, 0, .2, 0, 20, 3)])
        frame = decode_cloud(msg, 'front', 'radar_link', 0)
        self.assertEqual(len(frame.points), 1)
        self.assertEqual(len(Filter().update(frame)), 1)
        self.assertEqual(Filter(min_snr=1).update(frame), [])

    def test_invalid_schema_and_frame(self):
        msg = cloud([(2, 0, 0, 0, 10, 3)])
        with self.assertRaises(ValueError):
            decode_cloud(msg, 'front', 'wrong', 0)
        msg.data = msg.data[:-1]
        with self.assertRaises(ValueError):
            decode_cloud(msg, 'front', 'radar_link', 0)
        msg = cloud([(2, 0, 0, 0, 10, 3)])
        msg.fields[3].datatype = 8
        with self.assertRaises(ValueError):
            decode_cloud(msg, 'front', 'radar_link', 0)

    def test_empty_and_doppler_alias(self):
        self.assertEqual(decode_cloud(cloud([]), 'front', 'radar_link', 0).points, [])
        msg = cloud([(2, 0, 0, -.4, 10, 3)])
        msg.fields[3].name = 'doppler'
        self.assertAlmostEqual(decode_cloud(msg, 'front', 'radar_link', 0).points[0][3], -.4)
