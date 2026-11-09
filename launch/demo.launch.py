"""Experimental dry-output control; requires valid odometry, never the stationary default."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(get_package_share_directory('radar_navigation'))/'launch/system.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='reactive'),
        DeclareLaunchArgument('output', default_value='/radar/cmd_vel_safe'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
            launch_arguments={'fixed_frame': 'odom'}.items()),
        Node(package='radar_navigation', executable='radar_control',
            parameters=[{'mode': LaunchConfiguration('mode')}],
            remappings=[('/radar/cmd_vel_safe', LaunchConfiguration('output'))], output='screen')])
