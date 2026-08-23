# ============================================================================
# pickme-perception: RTAB-Map Visual Odometry and 3D SLAM
# ============================================================================
# Contains: RTAB-Map ROS 2 suite + pickme_bringup perception launch
# Rebuild:  When perception configurations or launch files change
# ============================================================================

FROM pickme-base

USER root

# ── System / ROS dependencies for perception & RTAB-Map ──
RUN apt-get update && apt-get install -y \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-odom \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap-util \
    ros-humble-rtabmap-sync \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# ── Configure dynamic linker for ROS and RTAB-Map shared libraries ──
RUN echo "/opt/ros/humble/lib" > /etc/ld.so.conf.d/ros.conf && \
    echo "/opt/ros/humble/lib/aarch64-linux-gnu" >> /etc/ld.so.conf.d/ros.conf && \
    echo "/opt/ros/humble/lib/x86_64-linux-gnu" >> /etc/ld.so.conf.d/ros.conf && \
    ldconfig

# ── Copy and build pickme_bringup ──
COPY src/pickme_bringup /tmp/perception_ws/src/pickme_bringup

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/perception_ws && \
        colcon build --install-base /opt/ros/humble --merge-install \
            --packages-select pickme_bringup" && \
    rm -rf /tmp/perception_ws && \
    ldconfig

USER developer
WORKDIR /home/developer

ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch pickme_bringup perception.launch.py"]
