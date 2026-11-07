"""Start the radar driver and publish where the radar sits on the robot.

  ros2 launch awr1843_radar radar.launch.py
  ros2 launch awr1843_radar radar.launch.py cfg_file:=/path/to/other.cfg mount_file:=/path/to/mount.yaml
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG = os.path.join(get_package_share_directory('awr1843_radar'), 'config')


def nodes(context):
    mount_file = LaunchConfiguration('mount_file').perform(context)
    with open(mount_file) as f:
        mount = yaml.safe_load(f)
    angles = [str(math.radians(mount[k])) for k in ('roll_deg', 'pitch_deg', 'yaw_deg')]
    actions = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='radar_tf',
             arguments=['--x', str(mount['x']), '--y', str(mount['y']), '--z', str(mount['z']),
                        '--roll', angles[0], '--pitch', angles[1], '--yaw', angles[2],
                        '--frame-id', 'base_footprint', '--child-frame-id', 'radar_link']),
        Node(package='awr1843_radar', executable='awr1843_driver', name='awr1843_driver',
             output='screen',
             parameters=[{'cfg_file': LaunchConfiguration('cfg_file').perform(context),
                          'frame_id': 'radar_link'}]),
    ]
    if not mount.get('measured', False):
        actions.insert(0, LogInfo(msg=f'WARNING: the radar position in {mount_file} is a placeholder. '
                                      'Measure it on the robot, then set measured: true.'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cfg_file', default_value=os.path.join(CONFIG, 'awr1843_profile.cfg'),
                              description='radar settings sent at startup'),
        DeclareLaunchArgument('mount_file', default_value=os.path.join(CONFIG, 'radar_mount.yaml'),
                              description='where the radar sits on the robot'),
        OpaqueFunction(function=nodes),
    ])
