# ============================================================================
# pickme-dev: Developer sandbox for writing and testing mission code
# ============================================================================
# Contains: ROS 2 Humble + px4_msgs + pickme_interfaces. src/ mounted via volume.
# Usage:    docker exec -it pickme-dev bash → colcon build → ros2 launch ...
# ============================================================================

FROM pickme-base

USER developer
WORKDIR /home/developer/workspace

CMD ["tail", "-f", "/dev/null"]
