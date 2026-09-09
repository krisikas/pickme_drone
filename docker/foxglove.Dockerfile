# ============================================================================
# pickme-foxglove: Lightweight telemetry & UI bridge
# ============================================================================
# Contains: Foxglove bridge and pickme_interfaces for topic introspection
# Rebuild:  Only when base or interfaces change
# ============================================================================

FROM pickme-base

USER root

RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace Setup & Build ──
WORKDIR /ws
COPY --chown=developer:developer src/pickme_bringup /ws/src/pickme_bringup
RUN chown -R developer:developer /ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ws && colcon build --symlink-install \
        --packages-select pickme_bringup"

USER developer
WORKDIR /home/developer

EXPOSE 8765

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && if [ ! -f /ws/install/setup.bash ]; then cd /ws && colcon build --symlink-install; fi && source /ws/install/setup.bash && exec ros2 launch pickme_bringup foxglove.launch.py"]
