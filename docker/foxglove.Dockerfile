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

COPY src/pickme_bringup /tmp/foxglove_ws/src/pickme_bringup

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/foxglove_ws && \
        colcon build --install-base /opt/ros/humble --merge-install" && \
    rm -rf /tmp/foxglove_ws

USER developer
WORKDIR /home/developer

EXPOSE 8765

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch pickme_bringup foxglove.launch.py"]
