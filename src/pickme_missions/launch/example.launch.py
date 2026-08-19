from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pickme_missions',
            executable='example_mission',
            name='example_mission',
            output='screen',
            emulate_tty=True,
        ),
    ])
