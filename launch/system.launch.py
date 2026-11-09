"""Stationary hardware acquisition, one measured mount, perception and optional viewer."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def system(context):
    mount_path = LaunchConfiguration('mount_file').perform(context)
    mount = yaml.safe_load(Path(mount_path).read_text())
    keys = ('x', 'y', 'z', 'roll_deg', 'pitch_deg', 'yaw_deg')
    if not mount.get('measured') or not all(math.isfinite(float(mount[k])) for k in keys):
        raise ValueError('A measured, finite radar mount is required')
    if abs(mount['roll_deg']) > 1 or abs(mount['pitch_deg']) > 1:
        raise ValueError('Current perception supports planar mounting; remeasure/align roll and pitch')
    tf = Node(package='tf2_ros', executable='static_transform_publisher', name='radar_tf',
        arguments=['--x', str(mount['x']), '--y', str(mount['y']), '--z', str(mount['z']),
                   '--roll', str(math.radians(mount['roll_deg'])),
                   '--pitch', str(math.radians(mount['pitch_deg'])),
                   '--yaw', str(math.radians(mount['yaw_deg'])),
                   '--frame-id', 'base_footprint', '--child-frame-id', 'radar_link'])
    driver = Node(package='radar_navigation', executable='awr1843_driver', name='awr1843_driver',
        output='screen', parameters=[{
            key: ParameterValue(LaunchConfiguration(key), value_type=str)
            for key in ('cli_port', 'data_port', 'cfg_file')}])
    perception = Node(package='radar_navigation', executable='radar_node', namespace='radar_front',
        output='screen', parameters=[{'input_topic': '/radar/points', 'frame_id': 'radar_link',
            'fixed_frame': ParameterValue(LaunchConfiguration('fixed_frame'), value_type=str)}])
    # A viewer exit is optional; loss of acquisition, TF or perception stops this launch.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='Required stationary radar process exited'))]))
        for process in (tf, driver, perception)]
    viewer_executable = str(Path(get_package_prefix('radar_navigation'))/'lib/radar_navigation/radar_viewer')
    viewer = ExecuteProcess(cmd=[viewer_executable, '--ros',
        '--topic', '/radar/points', '--port', LaunchConfiguration('viewer_port')],
        condition=IfCondition(LaunchConfiguration('viewer')), output='screen')
    return [*handlers, tf, driver, perception, viewer]


def generate_launch_description():
    share = Path(get_package_share_directory('radar_navigation'))
    defaults = dict(cli_port='', data_port='', cfg_file=str(share/'config/awr1843_profile.cfg'),
        mount_file=str(share/'config/radar_mount.yaml'), fixed_frame='base_footprint',
        viewer='false', viewer_port='8765')
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        OpaqueFunction(function=system)])
