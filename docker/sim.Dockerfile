FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 1. Install base build dependencies and Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    ca-certificates gnupg lsb-release sudo wget curl git cmake build-essential \
    python3-pip python-is-python3 gdb \
    astyle lcov libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav ninja-build protobuf-compiler \
    libeigen3-dev libopencv-dev rsync libarmadillo-dev libnlopt-cxx-dev libpcl-dev \
    && curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# 2. PX4 Python dependencies
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    empy==3.3.4 future pyros-genmsg kconfiglib jsonschema jinja2 pyserial pyyaml packaging toml numpy pandas

# 3. Build Micro-XRCE-DDS Agent & MAVLink-router
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/dds-agent && \
    cd /tmp/dds-agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib/ && \
    rm -rf /tmp/dds-agent

RUN apt-get update && apt-get install -y meson ninja-build pkg-config && \
    git clone https://github.com/mavlink-router/mavlink-router.git /tmp/mavlink-router && \
    cd /tmp/mavlink-router && \
    git submodule update --init --recursive && \
    meson setup build && \
    ninja -C build install && \
    rm -rf /tmp/mavlink-router

# 4. Set up non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# 5. Clone and compile PX4-Autopilot.
#    'make px4_sitl' compiles only, does NOT launch the simulator.
#    The gz model is specified at runtime in the entrypoint.
RUN git clone -b release/1.15 https://github.com/PX4/PX4-Autopilot.git --recursive && \
    cd PX4-Autopilot && \
    git checkout v1.15.4 && \
    git submodule update --init --recursive && \
    make px4_sitl

# 6. Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Start Micro-XRCE-DDS Agent in background\n\
MicroXRCEAgent udp4 -p 8888 &\n\
\n\
# Start MAVLink-router in background\n\
# mavlink-routerd requires a valid IP address, so we resolve host.docker.internal\n\
HOST_IP=$(getent hosts host.docker.internal | awk '\''{ print $1 }'\'' | head -n 1)\n\
if [ -n "$HOST_IP" ]; then\n\
    mavlink-routerd -e ${HOST_IP}:14550 127.0.0.1:14550 &\n\
fi\n\
\n\
export PX4_GZ_STANDALONE=1\n\
export PX4_SIM_MODEL=${PX4_SIM_MODEL:-gz_x500}\n\
export GZ_SIM_RESOURCE_PATH=/home/developer/PX4-Autopilot/Tools/simulation/gz/models:/home/developer/PX4-Autopilot/Tools/simulation/gz/worlds\${GZ_SIM_RESOURCE_PATH:+:\$GZ_SIM_RESOURCE_PATH}\n\
\n\
# Start Gazebo server (headless, no GUI) in background\n\
gz sim -s -r ${GZ_WORLD:-/home/developer/worlds/default.sdf} &\n\
GZ_PID=$!\n\
echo "Waiting for Gazebo server (PID $GZ_PID) to initialize..."\n\
sleep 5\n\
\n\
# Start PX4 SITL - connects to local Gazebo and spawns drone model\n\
cd /home/developer/PX4-Autopilot\n\
make px4_sitl ${PX4_SIM_MODEL}\n\
' > /home/$USERNAME/entrypoint.sh && chmod +x /home/$USERNAME/entrypoint.sh

ENTRYPOINT ["/home/developer/entrypoint.sh"]
