FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add official Gazebo repository and install pre-built Harmonic bridge package for Humble
RUN apt-get update && apt-get install -y \
    curl lsb-release gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Set up non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# Configure bash environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Run the ros_gz bridge for clock and sensor topics (camera, depth camera, pointcloud)
# Remap to unified topic contract matching RealSense D435
CMD ["/bin/bash", "-c", \
    "source /opt/ros/humble/setup.bash && \
     ros2 run ros_gz_bridge parameter_bridge \
       /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
       /camera@sensor_msgs/msg/Image[gz.msgs.Image \
       /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
       /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image \
       /depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
       /depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
       --ros-args \
       -r /camera:=/camera/color/image_raw \
       -r /camera/camera_info:=/camera/color/camera_info \
       -r /depth_camera:=/camera/depth/image_rect_raw \
       -r /depth_camera/camera_info:=/camera/depth/camera_info \
       -r /depth_camera/points:=/camera/depth/color/points"]
