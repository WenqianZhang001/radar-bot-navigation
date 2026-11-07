"""ROS 2 driver for the TI AWR1843BOOST radar (mmWave SDK 3.x out-of-box demo firmware).

Publishes /radar/points (sensor_msgs/PointCloud2) once per radar frame, in the radar_link frame
(ROS axes: x forward out of the antennas, y left, z up). Fields per point, all float32:
  x, y, z    position in meters
  velocity   radial velocity in m/s, positive = moving away from the radar
  snr, noise detection signal-to-noise ratio and noise level in dB

Parameters:
  cli_port, data_port  serial ports; empty = find the radar's XDS110 USB ports automatically
  cfg_file             radar settings sent at startup (default: this package's config)
  frame_id             frame of the published points (default radar_link)
  time_offset          seconds subtracted from the timestamps, for the radar's measurement delay
"""
import os
import signal

import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .radar_device import FrameParser, RadarCli, find_ports

FIELDS = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, name in enumerate(('x', 'y', 'z', 'velocity', 'snr', 'noise'))]


class Awr1843Driver(Node):
    def __init__(self):
        super().__init__('awr1843_driver')
        default_cfg = os.path.join(get_package_share_directory('awr1843_radar'),
                                   'config', 'awr1843_profile.cfg')
        cli_port = self.declare_parameter('cli_port', '').value
        data_port = self.declare_parameter('data_port', '').value
        cfg_file = self.declare_parameter('cfg_file', default_cfg).value
        self.frame_id = self.declare_parameter('frame_id', 'radar_link').value
        self.time_offset = Duration(seconds=self.declare_parameter('time_offset', 0.0).value)

        found_cli, found_data = find_ports()
        cli_port, data_port = cli_port or found_cli, data_port or found_data
        if not cli_port or not data_port:
            raise RuntimeError('radar not found in /dev/serial/by-id; check its USB cable and power')

        self.cli = RadarCli(cli_port)
        self.data = serial.Serial(data_port, 921600, timeout=0, exclusive=True)
        self.get_logger().info(f'configuring radar from {cfg_file}')
        for line, reply in self.cli.configure(cfg_file):
            self.get_logger().error(f'radar rejected "{line}": {reply}')
        self.data.reset_input_buffer()

        self.parser = FrameParser()
        self.pub = self.create_publisher(PointCloud2, 'radar/points', 10)
        self.last_frame = self.get_clock().now()
        self.create_timer(0.01, self.read_data)      # frames arrive at the radar's frame rate (10 Hz)
        self.create_timer(1.0, self.check_alive)
        self.get_logger().info(f'radar running; publishing radar/points in frame {self.frame_id}')

    def read_data(self):
        waiting = self.data.in_waiting
        if not waiting:
            return
        for _frame_num, points in self.parser.feed(self.data.read(waiting)):
            now = self.get_clock().now()
            header = Header(stamp=(now - self.time_offset).to_msg(), frame_id=self.frame_id)
            # TI axes (x right, y forward, z up) -> ROS axes (x forward, y left, z up)
            cloud = [(y, -x, z, v, snr, noise) for x, y, z, v, snr, noise in points]
            self.pub.publish(point_cloud2.create_cloud(header, FIELDS, cloud))
            self.last_frame = now

    def check_alive(self):
        silent = (self.get_clock().now() - self.last_frame).nanoseconds / 1e9
        if silent > 2.0:
            self.get_logger().warn(f'no radar frames for {silent:.0f} s; check the radar power',
                                   throttle_duration_sec=10.0)

    def stop_radar(self):
        try:
            self.cli.command('sensorStop')
        finally:
            self.cli.close()
            self.data.close()


def main():
    rclpy.init()
    try:
        node = Awr1843Driver()
    except (RuntimeError, OSError, serial.SerialException) as e:
        rclpy.logging.get_logger('awr1843_driver').fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C reaches this node twice (from the terminal and again from ros2 launch);
        # don't let the second one cut off the radar shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop_radar()
        node.get_logger().info('radar stopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
