from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='buff_detector',
            executable='buff_detector_node',
            name='buff_detector_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level','armor_detector:=debug'],
        ),
    ])
