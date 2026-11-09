"""Real odometry + radar mapping + reactive dry output by default; explicit actuation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('radar_navigation'))/'launch'
    return LaunchDescription([
        DeclareLaunchArgument('controller',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'base.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'system.launch.py')),
            launch_arguments={'fixed_frame':'odom'}.items()),
        Node(package='radar_navigation',executable='radar_control',output='screen',
            condition=IfCondition(LaunchConfiguration('controller')),
            parameters=[{'mode':'reactive','max_speed':.05,'max_turn':.2}])])
