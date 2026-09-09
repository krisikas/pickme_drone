import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('pickme_bringup')
    default_params_file = os.path.join(bringup_dir, 'config', 'core_params.yaml')

    default_use_sim_time = os.environ.get('USE_SIM_TIME', 'false').lower() in ('true', '1', 'yes')
    use_sim_time = LaunchConfiguration('use_sim_time', default=str(default_use_sim_time).lower())

    default_publish_camera_tf = os.environ.get('PUBLISH_CAMERA_TF', 'true').lower() in ('true', '1', 'yes')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf', default=str(default_publish_camera_tf).lower())

    default_launch_offboard = os.environ.get('LAUNCH_OFFBOARD', 'true').lower() in ('true', '1', 'yes')
    launch_offboard = LaunchConfiguration('launch_offboard', default=str(default_launch_offboard).lower())

    params_file = LaunchConfiguration('params_file', default=default_params_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(default_use_sim_time).lower(),
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'publish_camera_tf',
            default_value=str(default_publish_camera_tf).lower(),
            description='Broadcast static camera optical frames (set false on real drone)',
        ),
        DeclareLaunchArgument(
            'launch_offboard',
            default_value=str(default_launch_offboard).lower(),
            description='Launch offboard commander node (set false on real drone for bench testing)',
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
                {
                    'use_sim_time': use_sim_time,
                    'publish_camera_tf': publish_camera_tf,
                },
            ],
        ),
        Node(
            package='pickme_offboard',
            executable='offboard_node',
            name='offboard_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(launch_offboard),
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
