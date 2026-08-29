import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('pickme_bringup')
    default_params_file = os.path.join(bringup_dir, 'config', 'perception_params.yaml')
    default_slam = os.environ.get('SLAM_ENABLE', 'false').lower() in ('true', '1', 'yes')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default=str(default_slam).lower())
    publish_tf = LaunchConfiguration('publish_tf', default='false')
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    rgb_topic = LaunchConfiguration('rgb_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth/image_rect_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/color/camera_info')
    odom_topic = LaunchConfiguration('odom_topic', default='/visual_odometry')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value=str(default_slam).lower(),
            description='Run full RTAB-Map SLAM if true, otherwise run only Visual Odometry',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish odom -> base_link TF directly from VO (set false when fused via PX4 EKF2)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to YAML parameter file for perception and RTAB-Map',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_rect_raw',
            description='Rectified depth image topic',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera info topic',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/visual_odometry',
            description='Target topic for visual odometry output',
        ),

        # 1. RTAB-Map RGB-D Visual Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'publish_tf': publish_tf,
                },
            ],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('odom', odom_topic),
            ],
        ),

        # 2. RTAB-Map SLAM Node (Optional, enabled when slam:=true or SLAM_ENABLE=true)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(slam),
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('odom', odom_topic),
            ],
            arguments=['-d'],  # Delete database on startup for a clean session
        ),
    ])
