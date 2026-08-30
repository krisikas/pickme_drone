# ============================================================================
# pickme-base: Foundation image for all ROS 2 containers in PickmeDrone
# ============================================================================
# Contains: ROS 2 Humble + px4_msgs + pickme_interfaces + common libraries
# Rebuild:  When interfaces change or ROS dependencies are updated (~monthly)
# Usage:    Other Dockerfiles inherit via "FROM pickme-base"
# ============================================================================

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ── System packages ──
RUN apt-get update && apt-get install -y \
    git wget curl \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# ── Build px4_msgs into /opt/ros/humble overlay ──
RUN mkdir -p /tmp/px4_ws/src && \
    git clone -b release/1.16 https://github.com/PX4/px4_msgs.git /tmp/px4_ws/src/px4_msgs && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/px4_ws && \
        colcon build --install-base /opt/ros/humble --merge-install" && \
    rm -rf /tmp/px4_ws

# ── Build pickme_interfaces into /opt/ros/humble overlay ──
COPY src/pickme_interfaces /tmp/iface_ws/src/pickme_interfaces
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /tmp/iface_ws && \
        colcon build --install-base /opt/ros/humble --merge-install" && \
    rm -rf /tmp/iface_ws

# ── Create non-root user ──
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# ── Shell environment ──
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo 'source ~/workspace/install/setup.bash 2>/dev/null || true' >> ~/.bashrc

WORKDIR /home/$USERNAME/workspace
