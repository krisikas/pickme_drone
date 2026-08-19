#!/bin/bash
set -e

# 1. Start Micro-XRCE-DDS Agent in background (ROS 2 DDS bridge)
MicroXRCEAgent udp4 -p 8888 &

# 2. Start MAVLink-router in background as TCP server on port 5760
# Routes PX4's internal MAVLink stream (127.0.0.1:14550) to TCP port 5760 for QGroundControl
mavlink-routerd -t 5760 127.0.0.1:14550 &

# 3. Gazebo & PX4 Environment Setup
export GZ_PARTITION=${GZ_PARTITION:-sim}
export PX4_SIM_MODEL=${PX4_SIM_MODEL:-gz_x500_depth}

# 4. Start PX4 SITL in Headless mode (PX4 natively launches Gazebo with full timesync & official models)
cd /home/developer/PX4-Autopilot
HEADLESS=1 make px4_sitl ${PX4_SIM_MODEL}
