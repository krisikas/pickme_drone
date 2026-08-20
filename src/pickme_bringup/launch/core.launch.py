import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('pickme_bringup')
    default_params_file = os.path.join(bringup_dir, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=default_params_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to YAML parameter file',
        ),
        Node(
            package='pickme_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='pickme_offboard',
            executable='offboard_node',
            name='offboard_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
