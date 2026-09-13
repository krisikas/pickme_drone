# ============================================================================
# pickme-fcu: Dedicated Flight Controller (FCU) Hardware Bridge
# ============================================================================
# Contains: Micro-XRCE-DDS Agent v2.4.3 + MAVLink Router v3
# Purpose:  Bridges physical PX4 hardware (e.g., Matek H743 over UART /dev/ttyS0)
#           to ROS 2 DDS network and forwards MAVLink telemetry to QGroundControl.
# Isolation: Lives in its own container so restarting core/perception NEVER drops
#            the physical serial communication with the flight controller.
# ============================================================================

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ── Build Dependencies ──
RUN apt-get update && apt-get install -y \
    ca-certificates build-essential cmake git \
    meson ninja-build pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ── Build Micro-XRCE-DDS Agent v2.4.3 ──
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/dds-agent && \
    cd /tmp/dds-agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib/ && \
    rm -rf /tmp/dds-agent

# ── Build MAVLink-Router v3 ──
RUN git clone -b v3 https://github.com/mavlink-router/mavlink-router.git /tmp/mavlink-router && \
    cd /tmp/mavlink-router && \
    git submodule update --init --recursive && \
    meson setup build -Dsystemdsystemunitdir="" && \
    ninja -C build install && \
    rm -rf /tmp/mavlink-router

# ── Entrypoint Script ──
COPY docker/entrypoints/fcu-entrypoint.sh /usr/local/bin/fcu-entrypoint.sh
RUN chmod +x /usr/local/bin/fcu-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/fcu-entrypoint.sh"]
