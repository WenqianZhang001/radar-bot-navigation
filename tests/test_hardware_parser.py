"""Regression checks for imported hardware framing and ambiguous discovery."""
import math
import struct
import unittest
from unittest.mock import patch

from radar_navigation.driver.radar_device import FrameParser, MAGIC, find_ports


class HardwareParserTests(unittest.TestCase):
    def test_fragmented_packet_and_quality(self):
        body = struct.pack('<2I4f', 1, 16, .3, 2., .2, .4)
        body += struct.pack('<2I2h', 7, 4, 150, 30)
        packet = MAGIC + struct.pack('<8I', 1, 40+len(body), 0, 7, 0, 1, 2, 0) + body
        for cut in range(len(packet)+1):
            parser = FrameParser()
            result = parser.feed(packet[:cut]) + parser.feed(packet[cut:])
            self.assertEqual(len(result), 1)
            number, points = result[0]
            self.assertEqual(number, 7)
            for actual, expected in zip(points[0], (.3, 2., .2, .4, 15., 3.)):
                self.assertAlmostEqual(actual, expected, places=5)

    def test_empty_and_missing_quality(self):
        empty = MAGIC + struct.pack('<8I', 1, 40, 0, 1, 0, 0, 0, 0)
        body = struct.pack('<2I4f', 1, 16, 0., 1., 0., 0.)
        packet = MAGIC + struct.pack('<8I', 1, 64, 0, 2, 0, 1, 1, 0) + body
        result = FrameParser().feed(empty+packet)
        self.assertEqual(result[0], (1, []))
        self.assertTrue(all(math.isnan(v) for v in result[1][1][0][4:]))

    def test_multiple_radars_rejected(self):
        with patch('radar_navigation.driver.radar_device.glob.glob',
                   side_effect=[['cli-a', 'cli-b'], ['data-a', 'data-b']]):
            with self.assertRaisesRegex(RuntimeError, 'multiple XDS110'):
                find_ports()
