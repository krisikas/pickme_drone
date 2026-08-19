# ============================================================================
# pickme-dev: Developer sandbox for writing and testing mission code
# ============================================================================
# Contains: Dev tools (gdb, tmux, rqt, foxglove). src/ mounted via volume.
# Rebuild:  When you need new apt/pip packages (edit the bottom section)
# Usage:    docker exec -it pickme-dev bash → colcon build → ros2 launch ...
# ============================================================================

FROM pickme-base

USER root

# ── Dev tools ──
RUN apt-get update && apt-get install -y \
    gdb tmux vim nano htop \
    ros-humble-rqt ros-humble-rqt-common-plugins \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# ═══════════════════════════════════════════════════════════════════════
# QUICK-ADD SECTION: Need a new package? Add it here, then:
#   docker compose build dev && docker compose up -d dev
# Docker layer cache makes this rebuild take only seconds.
# ═══════════════════════════════════════════════════════════════════════
# RUN apt-get update && apt-get install -y \
#     ros-humble-foxglove-bridge \
#     && rm -rf /var/lib/apt/lists/*
# ═══════════════════════════════════════════════════════════════════════

USER developer
WORKDIR /home/developer/workspace

CMD ["tail", "-f", "/dev/null"]
