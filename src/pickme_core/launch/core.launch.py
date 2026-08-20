from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pickme_core',
            executable='transform_node',
            name='transform_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='pickme_core',
            executable='commander_node',
            name='commander_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
            }],
        ),
    ])
