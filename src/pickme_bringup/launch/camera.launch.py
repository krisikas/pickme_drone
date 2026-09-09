import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_camera_type = os.environ.get('CAMERA_TYPE', 'realsense').lower()
    default_rgb_width = os.environ.get('RGB_WIDTH', '640')
    default_rgb_height = os.environ.get('RGB_HEIGHT', '480')
    default_rgb_fps = os.environ.get('RGB_FPS', '30')
    default_depth_width = os.environ.get('DEPTH_WIDTH', '640')
    default_depth_height = os.environ.get('DEPTH_HEIGHT', '480')
    default_depth_fps = os.environ.get('DEPTH_FPS', '30')

    camera_type = LaunchConfiguration('camera_type', default=default_camera_type)
    rgb_width = LaunchConfiguration('rgb_width', default=default_rgb_width)
    rgb_height = LaunchConfiguration('rgb_height', default=default_rgb_height)
    rgb_fps = LaunchConfiguration('rgb_fps', default=default_rgb_fps)
    depth_width = LaunchConfiguration('depth_width', default=default_depth_width)
    depth_height = LaunchConfiguration('depth_height', default=default_depth_height)
    depth_fps = LaunchConfiguration('depth_fps', default=default_depth_fps)

    # Dynamic camera profile strings: e.g. "640x480x30"
    realsense_rgb_profile = PythonExpression(["'", rgb_width, "x", rgb_height, "x", rgb_fps, "'"])
    realsense_depth_profile = PythonExpression(["'", depth_width, "x", depth_height, "x", depth_fps, "'"])

    is_realsense = EqualsSubstitution(camera_type, 'realsense')
    is_oakd = EqualsSubstitution(camera_type, 'oakd')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type',
            default_value=default_camera_type,
            description='Camera driver backend: "realsense" or "oakd"',
        ),
        DeclareLaunchArgument(
            'rgb_width',
            default_value=default_rgb_width,
            description='RGB stream width in pixels',
        ),
        DeclareLaunchArgument(
            'rgb_height',
            default_value=default_rgb_height,
            description='RGB stream height in pixels',
        ),
        DeclareLaunchArgument(
            'rgb_fps',
            default_value=default_rgb_fps,
            description='RGB stream frame rate',
        ),
        DeclareLaunchArgument(
            'depth_width',
            default_value=default_depth_width,
            description='Depth stream width in pixels',
        ),
        DeclareLaunchArgument(
            'depth_height',
            default_value=default_depth_height,
            description='Depth stream height in pixels',
        ),
        DeclareLaunchArgument(
            'depth_fps',
            default_value=default_depth_fps,
            description='Depth stream frame rate',
        ),

        # ── 1. Intel RealSense Driver Node ────────────────────────────────────
        # Note: Do NOT set namespace='camera' here, as realsense2_camera internally
        # prefixes topics and frames with camera_name ('camera'), which would cause /camera/camera/...
        GroupAction(
            condition=IfCondition(is_realsense),
            actions=[
                Node(
                    package='realsense2_camera',
                    executable='realsense2_camera_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'camera_name': 'camera',
                        'base_frame_id': 'camera_link',
                        'depth_module.profile': realsense_depth_profile,
                        'rgb_camera.profile': realsense_rgb_profile,
                        'enable_color': True,
                        'enable_depth': True,
                        'align_depth.enable': True,
                        'pointcloud.enable': True,
                        'pointcloud.stream_filter': 2,
                        'pointcloud.stream_index_filter': 0,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'enable_sync': True,
                        'publish_tf': True,
                        'tf_publish_rate': 30.0,
                    }],
                    remappings=[
                        # Divert unaligned depth to avoid collision on /camera/depth/image_rect_raw
                        ('/camera/depth/image_rect_raw', '/camera/depth/image_raw_unaligned'),
                        ('/camera/depth/camera_info', '/camera/depth/camera_info_unaligned'),
                        # Route aligned depth directly into the contract topic for RTAB-Map
                        ('/camera/aligned_depth_to_color/image_raw', '/camera/depth/image_rect_raw'),
                        ('/camera/aligned_depth_to_color/camera_info', '/camera/depth/camera_info'),
                    ],
                ),
            ],
        ),

        # ── 2. Luxonis OAK-D Driver Node ──────────────────────────────────────
        GroupAction(
            condition=IfCondition(is_oakd),
            actions=[
                Node(
                    package='depthai_ros_driver',
                    executable='camera',
                    name='oak',
                    namespace='camera',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'camera.i_parent_frame': 'camera_link',
                        'camera.i_publish_tf_from_calibration': True,
                        'camera.i_nn_type': 'none',
                        'camera.i_pipeline_type': 'RGBD',
                        'rgb.i_resolution': '720P',
                        'rgb.i_fps': PythonExpression(['float(', rgb_fps, ')']),
                        'stereo.i_align_depth': True,
                        'stereo.i_publish_synced_rect_pair': True,
                    }],
                    remappings=[
                        ('rgb/image_raw', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('stereo/image_raw', '/camera/depth/image_rect_raw'),
                        ('stereo/camera_info', '/camera/depth/camera_info'),
                    ],
                ),
            ],
        ),
    ])
