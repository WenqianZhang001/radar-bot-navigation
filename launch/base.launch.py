"""Guarded OpenCR base and URDF only; no LiDAR, radar or controller."""
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('radar_navigation'))
    urdf=Path(get_package_share_directory('turtlebot3_description'))/'urdf/turtlebot3_waffle_pi.urdf'
    description=xacro.process_file(str(urdf),mappings={'namespace':''}).toxml()
    base=Node(package='radar_navigation',executable='base_watchdog',output='screen',
        arguments=['--port',LaunchConfiguration('opencr_port'),'--base-params',str(share/'config/waffle_pi_base.yaml'),
                   '--use-imu',LaunchConfiguration('odom_use_imu')],
        parameters=[{'actuate':ParameterValue(LaunchConfiguration('actuate'),value_type=bool)}])
    state=Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
        parameters=[{'robot_description':description}])
    return LaunchDescription([
        DeclareLaunchArgument('opencr_port',default_value='/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00'),
        DeclareLaunchArgument('odom_use_imu',default_value='false'),
        DeclareLaunchArgument('actuate',default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=base,on_exit=[EmitEvent(event=Shutdown(reason='Base watchdog stopped'))])),
        state,base])
