"""Single-radar Jetson path with explicit measured mount and dry command output."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share=FindPackageShare('radar_navigation')
    args=[DeclareLaunchArgument(k) for k in ('cli','data','x','y','z','yaw')]
    args += [DeclareLaunchArgument('mode',default_value='reactive'),
             DeclareLaunchArgument('output',default_value='/radar/cmd_vel_safe')]
    return LaunchDescription(args+[
        Node(package='tf2_ros',executable='static_transform_publisher',
             arguments=['--x',LaunchConfiguration('x'),'--y',LaunchConfiguration('y'),
                        '--z',LaunchConfiguration('z'),'--yaw',LaunchConfiguration('yaw'),
                        '--frame-id','base_link','--child-frame-id','radar_front']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share,'launch','radar.launch.py'])),
            launch_arguments={'cli':LaunchConfiguration('cli'),
                              'data':LaunchConfiguration('data'),
                              'cfg':PathJoinSubstitution([share,'awr1843_profile.cfg'])}.items()),
        Node(package='radar_navigation',executable='radar_control',
             parameters=[{'mode':LaunchConfiguration('mode')}],
             remappings=[('/radar/cmd_vel_safe',LaunchConfiguration('output'))],
             output='screen')
    ])
