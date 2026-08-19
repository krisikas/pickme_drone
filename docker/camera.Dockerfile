# ============================================================================
# pickme-camera: Intel RealSense depth camera driver
# ============================================================================
# Contains: librealsense2 SDK + realsense-ros ROS 2 wrapper
# Rebuild:  When changing camera model or updating SDK
# Hardware: /dev/bus/usb (USB bus for camera)
# Publishes: /camera/color/image_raw, /camera/depth/image_rect_raw,
#            /camera/depth/color/points
# ============================================================================

FROM pickme-base

USER root

# ── Install Intel RealSense SDK from official repo ──
RUN apt-get update && apt-get install -y \
    software-properties-common apt-transport-https \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
        | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
        https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Build realsense-ros from source (humble branch) ──
RUN mkdir -p /tmp/rs_ws/src && \
    git clone -b ros2 https://github.com/IntelRealSense/realsense-ros.git /tmp/rs_ws/src/realsense-ros && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/rs_ws && \
        rosdep update && \
        rosdep install --from-paths src --ignore-src -r -y && \
        colcon build --install-base /opt/ros/humble --merge-install \
            --packages-up-to realsense2_camera realsense2_description" && \
    rm -rf /tmp/rs_ws

USER developer
WORKDIR /home/developer

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py"]
