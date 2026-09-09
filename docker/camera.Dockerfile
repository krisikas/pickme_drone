# ============================================================================
# pickme-camera: Real Depth & RGB Camera Driver Container
# ============================================================================
# Contains: Intel RealSense (realsense2_camera) + Luxonis DepthAI (depthai_ros)
# Purpose:  Interfaces directly with physical camera hardware over USB 3.0,
#           publishing standardized ROS 2 image, depth, and camera_info topics.
# ============================================================================

FROM pickme-base

USER root

# ── Install RealSense and DepthAI ROS 2 drivers ──
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-depthai-ros-driver \
    ros-humble-depthai-bridge \
    ros-humble-depthai-descriptions \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    v4l-utils \
    udev \
    && rm -rf /var/lib/apt/lists/*

# ── Configure dynamic linker for shared libraries ──
RUN echo "/opt/ros/humble/lib" > /etc/ld.so.conf.d/ros.conf && \
    echo "/opt/ros/humble/lib/aarch64-linux-gnu" >> /etc/ld.so.conf.d/ros.conf && \
    echo "/opt/ros/humble/lib/x86_64-linux-gnu" >> /etc/ld.so.conf.d/ros.conf && \
    ldconfig

# ── Workspace Setup & Build ──
WORKDIR /ws
COPY --chown=developer:developer src/pickme_bringup /ws/src/pickme_bringup
RUN chown -R developer:developer /ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ws && colcon build --symlink-install \
        --packages-select pickme_bringup" && \
    ldconfig

USER developer
WORKDIR /home/developer

ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && if [ ! -f /ws/install/setup.bash ]; then cd /ws && colcon build --symlink-install; fi && source /ws/install/setup.bash && exec ros2 launch pickme_bringup camera.launch.py"]
