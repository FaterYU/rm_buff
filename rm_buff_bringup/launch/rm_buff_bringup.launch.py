import yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node


launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_buff_bringup'), 'config', 'launch_params.yaml')))

robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])


robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)

serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

detector_node = Node(
        package='buff_detector',
        executable='buff_detector_node',
        emulate_tty=True,
        output='both',
        arguments=['--ros-args', '--log-level',
                   'buff_detector:='+launch_params['detector_log_level']],
    )

tracker_node = Node(
        package='buff_tracker',
        executable='buff_tracker_node',
        emulate_tty=True,
        output='both',
        arguments=['--ros-args', '--log-level',
                   'buff_tracker:='+launch_params['tracker_log_level']],
    )

def generate_launch_description():
    return LaunchDescription([
        robot_state_publisher,
        detector_node,
        tracker_node,
        delay_serial_node,
    ])

