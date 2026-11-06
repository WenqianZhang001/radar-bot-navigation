"""One sensor per launch; use unique namespace, frame and serial paths for two."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(namespace='radar_front', cli='', data='', cfg='', replay='',
                    frame_id='radar_front', fixed_frame='odom')
    arguments = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    parameters = {k: ParameterValue(LaunchConfiguration(k), value_type=str) for k in defaults if k != 'namespace'}
    return LaunchDescription(arguments + [Node(package='radar_navigation',
        executable='radar_node', namespace=LaunchConfiguration('namespace'),
        parameters=[parameters], output='screen')])
