"""Stock TurtleBot3 child with independent command leases and Linux parent-death stop.

Only the native child opens OpenCR. It receives one private Twist stream. If this
watchdog dies, PR_SET_PDEATHSIG kills that child, removing its firmware heartbeat.
"""
import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .base_safety import BaseLease
from .base_coordinates import native_command


def spawn_base(command):
    parent = os.getpid()
    libc = ctypes.CDLL(None, use_errno=True)
    def bind_parent():
        if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:
            os._exit(125)
        if os.getppid() != parent:
            os.kill(os.getpid(), signal.SIGKILL)
    # Called before rclpy.init creates DDS threads; do not move below ROS init.
    return subprocess.Popen(command, preexec_fn=bind_parent)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--base-params', required=True)
    parser.add_argument('--use-imu', choices=['true','false'], default='false')
    args, ros_args = parser.parse_known_args()
    port = Path(args.port)
    if not port.exists() or subprocess.run(['fuser', str(port.resolve())], capture_output=True).returncode != 1:
        raise RuntimeError('OpenCR missing, already owned, or ownership check failed')
    from ament_index_python.packages import get_package_prefix
    executable = str(Path(get_package_prefix('turtlebot3_node'))/'lib/turtlebot3_node/turtlebot3_ros')
    child = spawn_base([executable, '-i', args.port, '--ros-args', '--params-file', args.base_params,
        '-p', 'odometry.use_imu:='+args.use_imu, '-r', 'cmd_vel:=/base/driver_cmd_vel',
        '-r','odom:=/base/native_odom','-r','joint_states:=/base/native_joint_states_raw',
        '-r','imu:=/base/native_imu_raw'])
    sys.argv = [sys.argv[0], *ros_args]
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rcl_interfaces.msg import ParameterDescriptor
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from turtlebot3_msgs.msg import SensorState
    from .lifecycle import initialize, begin_cleanup
    requested = initialize()
    node = None
    try:
        class Watchdog(Node):
            def __init__(self):
                super().__init__('base_watchdog')
                self.lease = BaseLease(actuate=self.declare_parameter('actuate', False,
                    ParameterDescriptor(read_only=True)).value,
                    max_speed=self.declare_parameter('max_speed', .05).value,
                    max_turn=self.declare_parameter('max_turn', .2).value)
                self.pub = self.create_publisher(Twist, '/base/driver_cmd_vel', 1)
                self.status = self.create_publisher(String, '/base/watchdog_status', 10)
                self.create_subscription(Twist, '/radar/cmd_vel_safe', self.command, 1)
                self.create_subscription(Bool, '/radar/enable', self.enable, 1)
                self.create_subscription(Odometry, '/odom', self.odom, 10)
                self.create_subscription(SensorState, '/sensor_state', self.hardware, 10)
                self.create_timer(.05, self.tick)
                self.get_logger().info(f'base child pid={child.pid}; actuate={self.lease.actuate}; kernel parent-death SIGKILL configured')

            def command(self, msg):
                unsupported = any(v != 0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y))
                self.lease.accept(msg.linear.x,msg.angular.z,time.monotonic(),unsupported)

            def enable(self, msg):
                self.lease.enable_at = time.monotonic() if msg.data else -float('inf')

            def odom(self, msg):
                age=(self.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
                values=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z]
                if msg.header.frame_id=='odom' and msg.child_frame_id=='base_footprint' and 0<=age<=.3 and all(math.isfinite(v) for v in values):
                    self.lease.odom_at=time.monotonic()-age

            def hardware(self, msg):
                age=(self.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
                self.lease.hardware_at = time.monotonic()-age if msg.torque and 10<=msg.battery<=15 and 0<=age<=.3 else -float('inf')

            def tick(self):
                if child.poll() is not None:
                    raise RuntimeError(f'TurtleBot3 child exited: {child.returncode}')
                if len(self.get_publishers_info_by_topic('/base/driver_cmd_vel'))>1 or len(self.get_publishers_info_by_topic('/radar/cmd_vel_safe'))>1:
                    raise RuntimeError('Competing base/control command publishers; stopping base heartbeat')
                now=time.monotonic(); linear,angular=self.lease.output(now)
                msg=Twist();msg.linear.x,msg.angular.z=native_command(linear,angular);self.pub.publish(msg)
                def age(value):return now-value if math.isfinite(value) else None
                self.status.publish(String(data=json.dumps(dict(reason=self.lease.reason,linear=linear,angular=angular,
                    actuate=self.lease.actuate,base_pid=child.pid,native_linear=msg.linear.x,native_angular=msg.angular.z,command_age=age(self.lease.command_at),
                    odom_age=age(self.lease.odom_at),enable_age=age(self.lease.enable_at),
                    hardware_age=age(self.lease.hardware_at)))))
        node=Watchdog()
        while rclpy.ok() and not requested.is_set():
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        begin_cleanup()
        if node is not None:
            for _ in range(5):
                node.pub.publish(Twist());time.sleep(.05)
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
