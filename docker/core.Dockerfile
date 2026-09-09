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
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace Setup & Build ──
WORKDIR /ws
COPY --chown=developer:developer src/pickme_odometry /ws/src/pickme_odometry
COPY --chown=developer:developer src/pickme_offboard /ws/src/pickme_offboard
COPY --chown=developer:developer src/pickme_bringup /ws/src/pickme_bringup
RUN chown -R developer:developer /ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ws && colcon build --symlink-install \
        --packages-select pickme_odometry pickme_offboard pickme_bringup"

USER developer
WORKDIR /home/developer

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && if [ ! -f /ws/install/setup.bash ]; then cd /ws && colcon build --symlink-install; fi && source /ws/install/setup.bash && exec ros2 launch pickme_bringup core.launch.py"]
