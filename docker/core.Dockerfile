# ============================================================================
# pickme-core: Stable background services (commander, localization, SLAM)
# ============================================================================
# Contains: Compiled pickme_commander (and future stable packages)
# Rebuild:  When commander or other core logic changes
# Usage:    Runs as a background service, does NOT restart with dev
# ============================================================================

FROM pickme-base

USER root

# ── Copy and build stable packages ──
COPY src/pickme_commander /tmp/core_ws/src/pickme_commander

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/core_ws && \
        colcon build --install-base /opt/ros/humble --merge-install \
            --packages-select pickme_commander" && \
    rm -rf /tmp/core_ws

RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

USER developer
WORKDIR /home/developer

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch pickme_commander commander.launch.py"]
