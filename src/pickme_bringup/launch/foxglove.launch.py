from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='8765')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8765',
            description='Foxglove websocket port',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            arguments=['--ros-args', '--log-level', 'foxglove_bridge:=error'],
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
