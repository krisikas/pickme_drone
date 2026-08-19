from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pickme_commander',
            executable='commander_node',
            name='commander_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
