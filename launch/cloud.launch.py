"""Stationary external-driver perception; no serial owner, mount TF, or control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    defaults = {'input_topic': '/radar/points', 'frame_id': 'radar_link',
                'fixed_frame': 'base_footprint'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='radar_navigation', executable='radar_node',
             namespace='radar_front', output='screen',
             parameters=[{k: LaunchConfiguration(k) for k in defaults}])])
