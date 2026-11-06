"""Prior-map/AMCL localization with radar local obstacles and gated commands.

No robot output by default. Requires a validated robot Nav2 parameter file
merged by scripts/prepare_nav2.py, a prior map, /scan, odometry and TF.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('output', default_value='/radar/cmd_vel_safe'),
        GroupAction([
            SetRemap(src='cmd_vel',dst='/radar/cmd_vel_request'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'),'launch','bringup_launch.py'])),
                launch_arguments={'map':LaunchConfiguration('map'),
                                  'params_file':LaunchConfiguration('params_file'),
                                  'use_sim_time':'false','use_composition':'False'}.items())
        ]),
        Node(package='radar_navigation',executable='radar_control',
             parameters=[{'mode':'external','require_localization':True}],
             remappings=[('/radar/cmd_vel_safe',LaunchConfiguration('output'))],
             output='screen')
    ])
