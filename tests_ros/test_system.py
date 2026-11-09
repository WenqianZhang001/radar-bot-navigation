"""Installed canonical launch with PTY sensor and subscriber-only viewer, no robot devices."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest
import urllib.request

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from radar_navigation.driver.radar_device import MAGIC


class StationarySystemTests(unittest.TestCase):
    def exercise(self, reject=False):
        rclpy.init()
        node = Node('canonical_system_test')
        source, processed, maps, status, transforms = [], [], [], [], []
        subscriptions = [
            node.create_subscription(PointCloud2, '/radar/points', source.append, 10),
            node.create_subscription(PointCloud2, '/radar_front/points', processed.append, 10),
            node.create_subscription(OccupancyGrid, '/radar_front/occupancy', maps.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(String, '/radar_front/status', status.append, 10),
            node.create_subscription(TFMessage, '/tf_static', transforms.append,
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        cli_master, cli_slave = pty.openpty()
        data_master, data_slave = pty.openpty()
        quit_ = threading.Event(); transmit = threading.Event(); running = threading.Event()
        transmit.set()
        commands = []
        def emulate():
            line = b''; number = 0; last = 0
            while not quit_.is_set():
                ready, _, _ = select.select([cli_master], [], [], .01)
                if ready:
                    for byte in os.read(cli_master, 4096):
                        if byte == 10:
                            command = line.strip().decode(); commands.append(command)
                            if command == 'sensorStart': running.set()
                            if command == 'sensorStop': running.clear()
                            reply = b'Error\nmmwDemo:/>' if command == 'badCommand' else b'Done\nmmwDemo:/>'
                            os.write(cli_master, reply); line = b''
                        else: line += bytes([byte])
                if running.is_set() and transmit.is_set() and time.monotonic()-last > .1:
                    body = struct.pack('<2I4f', 1, 16, .3, 2., .2, .4)
                    body += struct.pack('<2I2h', 7, 4, 150, 30)
                    packet = MAGIC + struct.pack('<8I', 1, 40+len(body), 0, number, 0, 1, 2, 0) + body
                    os.write(data_master, packet); number += 1; last = time.monotonic()
        thread = threading.Thread(target=emulate, daemon=True); thread.start()
        process = None
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode='w+') as log:
            try:
                cfg = Path(directory)/'profile.cfg'
                cfg.write_text('badCommand\nsensorStart\n' if reject else 'sensorStart\n')
                with socket.socket() as sock:
                    sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
                command = ['ros2', 'launch', 'radar_navigation', 'system.launch.py',
                    'cli_port:='+os.ttyname(cli_slave), 'data_port:='+os.ttyname(data_slave),
                    'cfg_file:='+str(cfg), 'viewer:='+('false' if reject else 'true'),
                    'viewer_port:='+str(port)]
                process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                def until(predicate, seconds=15):
                    deadline = time.monotonic()+seconds
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node, timeout_sec=.03)
                        if predicate(): return
                    self.fail('Canonical system condition timed out')
                if reject:
                    until(lambda: process.poll() is not None)
                    self.assertIn('sensorStop', commands)
                    log.seek(0); self.assertIn('radar rejected configuration', log.read())
                    return
                until(lambda: len(source)>5 and processed and maps and transforms)
                p = next(iter(read_points(processed[-1], field_names=('x','y','z','doppler'))))
                for actual, expected in zip(p, (2.15, -.24, .39, .4)):
                    self.assertAlmostEqual(float(actual), expected, places=5)
                self.assertEqual(processed[-1].header.frame_id, 'base_footprint')
                self.assertEqual([f.name for f in source[-1].fields], ['x','y','z','velocity','snr','noise'])
                self.assertEqual(len(node.get_publishers_info_by_topic('/tf_static')), 1)
                self.assertFalse(node.get_publishers_info_by_topic('/cmd_vel'))
                until(lambda: any(name=='radar_viewer' for name, _ in node.get_node_names_and_namespaces()))
                self.assertTrue(any(e.node_name=='radar_viewer' for e in node.get_subscriptions_info_by_topic('/radar/points')))
                with urllib.request.urlopen('http://127.0.0.1:'+str(port), timeout=3) as response:
                    self.assertIn(b'AWR1843 live point cloud', response.read())
                until(lambda: any(v>=65 for v in maps[-1].data))
                transmit.clear()
                until(lambda: maps and all(v==-1 for v in maps[-1].data) and status and json.loads(status[-1].data)['stale'], seconds=4)
                share = Path(get_package_share_directory('radar_navigation'))
                for file in ['config/radar_mount.yaml','config/awr1843_profile.cfg','config/stationary.rviz',
                             'scripts/analyze_spatial_experiment.py','scripts/analyze_spatial_expiry.py']:
                    self.assertTrue((share/file).is_file(), file)
            finally:
                if process and process.poll() is None:
                    os.kill(process.pid, signal.SIGINT)
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL); process.wait()
                quit_.set(); thread.join(timeout=1)
                for fd in (cli_master, cli_slave, data_master, data_slave): os.close(fd)
                node.destroy_node(); rclpy.shutdown()
                log.seek(0); output=log.read(); print(output[-6000:])
                self.assertIn('sensorStop', commands)
                if not reject:
                    self.assertEqual(process.returncode, 0)
                    self.assertNotIn('Traceback', output)
                    self.assertNotIn('escalating to', output)

    def test_complete_system_and_clean_stop(self):
        self.exercise()

    def test_rejected_profile_stops_complete_launch(self):
        self.exercise(reject=True)
