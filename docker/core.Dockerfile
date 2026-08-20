# ============================================================================
# pickme-core: Stable background services (commander, transforms, foxglove)
# ============================================================================
# Contains: Compiled pickme_core (and future stable packages)
# Rebuild:  When commander, transforms, or other core logic changes
# Usage:    Runs as a background service, does NOT restart with dev
# ============================================================================

FROM pickme-base

USER root

# ── System / ROS dependencies for core services ──
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── Copy and build stable core packages ──
COPY src/pickme_odometry /tmp/core_ws/src/pickme_odometry
COPY src/pickme_offboard /tmp/core_ws/src/pickme_offboard
COPY src/pickme_bringup /tmp/core_ws/src/pickme_bringup

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/core_ws && \
        colcon build --install-base /opt/ros/humble --merge-install \
            --packages-select pickme_odometry pickme_offboard pickme_bringup" && \
    rm -rf /tmp/core_ws

USER developer
WORKDIR /home/developer

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch pickme_bringup core.launch.py"]
