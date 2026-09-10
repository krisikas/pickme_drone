import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for camera driver (RealSense D435 / Luxonis OAK-D).

    Adheres strictly to ros2-engineering-skills:
    - REP-103 and REP-105 coordinate frame standards (camera_link as parent, optical RDF frames).
    - Unified project perception topic contract (/camera/color/..., /camera/depth/..., /camera/depth/color/points).
    - Eliminates duplicate namespace (/camera/camera) and duplicate TF prefix (camera_camera_link).
    - Disables high-bandwidth unused streams (IR1, IR2, IMU gyro/accel) to preserve embedded resources.
    """
    default_camera_type = os.environ.get('CAMERA_TYPE', 'realsense').lower()
    default_rgb_width = os.environ.get('RGB_WIDTH', '640')
    default_rgb_height = os.environ.get('RGB_HEIGHT', '480')
    default_rgb_fps = os.environ.get('RGB_FPS', '30')
    default_depth_width = os.environ.get('DEPTH_WIDTH', '640')
    default_depth_height = os.environ.get('DEPTH_HEIGHT', '480')
    default_depth_fps = os.environ.get('DEPTH_FPS', '30')
    default_pointcloud = os.environ.get('ENABLE_POINTCLOUD', 'true').lower() in ('true', '1', 'yes')
    default_publish_tf = os.environ.get('PUBLISH_TF', 'true').lower() in ('true', '1', 'yes')

    camera_type = LaunchConfiguration('camera_type', default=default_camera_type)
    rgb_width = LaunchConfiguration('rgb_width', default=default_rgb_width)
    rgb_height = LaunchConfiguration('rgb_height', default=default_rgb_height)
    rgb_fps = LaunchConfiguration('rgb_fps', default=default_rgb_fps)
    depth_width = LaunchConfiguration('depth_width', default=default_depth_width)
    depth_height = LaunchConfiguration('depth_height', default=default_depth_height)
    depth_fps = LaunchConfiguration('depth_fps', default=default_depth_fps)
    pointcloud_enable = LaunchConfiguration('pointcloud_enable', default=str(default_pointcloud).lower())
    publish_tf = LaunchConfiguration('publish_tf', default=str(default_publish_tf).lower())

    # Dynamic camera profile strings: e.g. "640x480x30"
    realsense_rgb_profile = PythonExpression(["'", rgb_width, "x", rgb_height, "x", rgb_fps, "'"])
    realsense_depth_profile = PythonExpression(["'", depth_width, "x", depth_height, "x", depth_fps, "'"])

    # Boolean expressions for driver parameter conversion
    enable_pointcloud_bool = PythonExpression(["'", pointcloud_enable, "'.lower() in ('true', '1', 'yes')"])
    publish_tf_bool = PythonExpression(["'", publish_tf, "'.lower() in ('true', '1', 'yes')"])

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
        DeclareLaunchArgument(
            'pointcloud_enable',
            default_value=str(default_pointcloud).lower(),
            description='Enable 3D point cloud stream generation',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value=str(default_publish_tf).lower(),
            description='Broadcast internal camera optical TF frames',
        ),

        # ── 1. Intel RealSense Driver Node ────────────────────────────────────
        # Frame hierarchy produced by realsense2_camera:
        #   base_frame_id = camera_name + '_' + 'link' -> 'camera_link' (FLU)
        #   color_optical_frame = 'camera_color_optical_frame' (RDF)
        #   depth_optical_frame = 'camera_depth_optical_frame' (RDF)
        # Setting base_frame_id to 'link' ensures it connects cleanly to robot's
        # base_link -> camera_link without the erroneous 'camera_camera_link'.
        GroupAction(
            condition=IfCondition(is_realsense),
            actions=[
                Node(
                    package='realsense2_camera',
                    executable='realsense2_camera_node',
                    name='camera',
                    namespace='',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'camera_name': 'camera',
                        'base_frame_id': 'link',
                        # RealSense ROS v4.58+ parameter names
                        'depth_module.depth_profile': realsense_depth_profile,
                        'rgb_camera.color_profile': realsense_rgb_profile,
                        # Backwards compatibility aliases
                        'depth_module.profile': realsense_depth_profile,
                        'rgb_camera.profile': realsense_rgb_profile,

                        'enable_color': True,
                        'enable_depth': True,
                        'align_depth.enable': True,
                        # Pointcloud parameters: support both x86 ('pointcloud.*') and ARM64 NEON ('pointcloud__neon_.*')
                        'pointcloud.enable': pointcloud_enable,
                        'pointcloud__neon_.enable': pointcloud_enable,
                        'pointcloud.stream_filter': 2,        # Color texture on pointcloud
                        'pointcloud__neon_.stream_filter': 2,
                        'pointcloud.stream_index_filter': 0,
                        'pointcloud__neon_.stream_index_filter': 0,
                        'enable_infra1': False,               # Disabled to conserve USB bandwidth
                        'enable_infra2': False,               # Disabled to conserve USB bandwidth
                        'enable_gyro': False,                 # Disabled: flight IMU owned by PX4
                        'enable_accel': False,                # Disabled: flight IMU owned by PX4
                        'enable_sync': True,                  # Hardware sync for RGB & Depth
                        'publish_tf': True,
                        'tf_publish_rate': 30.0,
                    }],
                    remappings=[
                        # 1. Divert unaligned depth to prevent topic collision with contract topic
                        ('/camera/depth/image_rect_raw', '/camera/depth/image_raw_unaligned'),
                        ('/camera/depth/camera_info', '/camera/depth/camera_info_unaligned'),
                        ('/camera/camera/depth/image_rect_raw', '/camera/depth/image_raw_unaligned'),
                        ('/camera/camera/depth/camera_info', '/camera/depth/camera_info_unaligned'),
                        ('depth/image_rect_raw', '/camera/depth/image_raw_unaligned'),
                        ('depth/camera_info', '/camera/depth/camera_info_unaligned'),

                        # 2. Route aligned depth to standard contract topic for RTAB-Map / VO
                        ('/camera/aligned_depth_to_color/image_raw', '/camera/depth/image_rect_raw'),
                        ('/camera/aligned_depth_to_color/camera_info', '/camera/depth/camera_info'),
                        ('/camera/camera/aligned_depth_to_color/image_raw', '/camera/depth/image_rect_raw'),
                        ('/camera/camera/aligned_depth_to_color/camera_info', '/camera/depth/camera_info'),
                        ('aligned_depth_to_color/image_raw', '/camera/depth/image_rect_raw'),
                        ('aligned_depth_to_color/camera_info', '/camera/depth/camera_info'),

                        # 3. Clean RGB image stream
                        ('/camera/camera/color/image_raw', '/camera/color/image_raw'),
                        ('/camera/camera/color/camera_info', '/camera/color/camera_info'),
                        ('color/image_raw', '/camera/color/image_raw'),
                        ('color/camera_info', '/camera/color/camera_info'),

                        # 4. Clean 3D Point Cloud stream
                        ('/camera/camera/depth/color/points', '/camera/depth/color/points'),
                        ('depth/color/points', '/camera/depth/color/points'),
                        ('camera/depth/color/points', '/camera/depth/color/points'),
                        ('pointcloud', '/camera/depth/color/points'),
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
                        'camera.i_publish_tf_from_calibration': publish_tf_bool,
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
