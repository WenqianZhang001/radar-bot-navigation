"""External PointCloud2 input through real ROS middleware; no serial devices."""
import json
import os
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster


class CloudIntegration(unittest.TestCase):
    def test_external_driver_to_map_and_expiry(self):
        rclpy.init()
        node = Node('cloud_input_observer')
        broadcaster = StaticTransformBroadcaster(node)
        mount = TransformStamped()
        mount.header.frame_id = 'base_footprint'
        mount.child_frame_id = 'radar_link'
        mount.transform.translation.x = .036
        mount.transform.translation.y = -.065
        mount.transform.translation.z = .185
        mount.transform.rotation.w = 1.
        broadcaster.sendTransform(mount)
        publisher = node.create_publisher(PointCloud2, '/radar/points', qos_profile_sensor_data)
        clouds, maps, metadata, health = [], [], [], []
        subscriptions = [
            node.create_subscription(PointCloud2, '/radar_front/points', clouds.append, 10),
            node.create_subscription(String, '/radar_front/metadata', metadata.append, 10),
            node.create_subscription(String, '/radar_front/status', health.append, 10),
            node.create_subscription(OccupancyGrid, '/radar_front/occupancy', maps.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'velocity', 'snr', 'noise'))]
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(['ros2', 'launch', 'radar_navigation', 'cloud.launch.py'],
                                       stdout=log, stderr=log, start_new_session=True)
            try:
                end = time.monotonic()+15
                while time.monotonic() < end:
                    publisher.publish(create_cloud(Header(stamp=node.get_clock().now().to_msg(),
                        frame_id='radar_link'), fields, [(2., .3, .2, .4, 15., 3.)]))
                    rclpy.spin_once(node, timeout_sec=.05)
                    if clouds and metadata and maps and any(v >= 65 for v in maps[-1].data):
                        break
                self.assertTrue(clouds, 'No processed cloud')
                self.assertTrue(metadata)
                self.assertTrue(maps and any(v >= 65 for v in maps[-1].data))
                point = next(iter(read_points(clouds[-1], field_names=('x','y','z','doppler'))))
                for actual, expected in zip(point, (2.036, .235, .385, .4)):
                    self.assertAlmostEqual(float(actual), expected, places=5)
                self.assertFalse(json.loads(metadata[-1].data)['metadata']['hardware_frame_number_available'])
                end = time.monotonic()+3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                self.assertTrue(all(v == -1 for v in maps[-1].data))
                self.assertTrue(json.loads(health[-1].data)['stale'])
                self.assertIsNone(process.poll())
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                log.seek(0)
                print(log.read())
                node.destroy_node()
                rclpy.shutdown()
