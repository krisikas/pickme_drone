import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_use_sim_time = os.environ.get('USE_SIM_TIME', 'false').lower() in ('true', '1', 'yes')
    use_sim_time = LaunchConfiguration('use_sim_time', default=str(default_use_sim_time).lower())
    default_port = os.environ.get('FOXGLOVE_PORT', '8765')
    port = LaunchConfiguration('port', default=default_port)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(default_use_sim_time).lower(),
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'port',
            default_value=default_port,
            description='Foxglove websocket port',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'foxglove_bridge:=error'],
            parameters=[{
                'port': port,
                'use_sim_time': use_sim_time,
                'send_buffer_limit': 10000000,  # Reduced from 100MB to 10MB to eliminate 20s bufferbloat latency
                'use_compression': False,  # Disabled: software zlib chokes weak Cortex-A55 cores on ARM SBC
            }],
        ),
    ])
