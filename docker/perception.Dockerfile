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

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && if [ ! -f /ws/install/setup.bash ]; then cd /ws && colcon build --symlink-install; fi && source /ws/install/setup.bash && exec ros2 launch pickme_bringup perception.launch.py"]
