# ============================================================================
# pickme-px4-bridge: Communication bridge to PX4 flight controller
# ============================================================================
# Contains: Micro-XRCE-DDS Agent (C++ utility, no ROS needed)
# Rebuild:  Almost never (only when DDS Agent version changes)
# Hardware: /dev/ttyACM0 (USB to flight controller)
# Critical: If this container dies, drone loses ROS↔PX4 link
# ============================================================================

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ── Build dependencies ──
RUN apt-get update && apt-get install -y \
    git cmake build-essential meson ninja-build pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ── Build Micro-XRCE-DDS Agent ──
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/dds-agent && \
    cd /tmp/dds-agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib/ && \
    rm -rf /tmp/dds-agent

# ── Build MAVLink-router ──
RUN git clone https://github.com/mavlink-router/mavlink-router.git /tmp/mavlink-router && \
    cd /tmp/mavlink-router && \
    git submodule update --init --recursive && \
    meson setup build && \
    ninja -C build install && \
    rm -rf /tmp/mavlink-router

# ── Remove build tools to shrink image ──
RUN apt-get purge -y git cmake build-essential meson ninja-build pkg-config && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

