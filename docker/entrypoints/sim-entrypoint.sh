#!/bin/bash
set -e

ENABLE_MAVLINK_ROUTER="${ENABLE_MAVLINK_ROUTER:-true}"
MAVLINK_TCP_PORT="${MAVLINK_TCP_PORT:-5760}"
MAVLINK_QGC_IP="${MAVLINK_QGC_IP:-127.0.0.1}"
MAVLINK_QGC_PORT="${MAVLINK_QGC_PORT:-14550}"

export GZ_PARTITION="${GZ_PARTITION:-sim}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500_depth}"
export GZ_IP="$(hostname -i 2>/dev/null | awk '{print $1}')"

MAVLINK_PID=""
AGENT_PID=""
PX4_PID=""

cleanup() {
    echo "Stopping simulation processes..."
    if [ -n "${PX4_PID}" ] && kill -0 "${PX4_PID}" 2>/dev/null; then
        kill -TERM "${PX4_PID}" 2>/dev/null || true
    fi
    if [ -n "${MAVLINK_PID}" ] && kill -0 "${MAVLINK_PID}" 2>/dev/null; then
        kill -TERM "${MAVLINK_PID}" 2>/dev/null || true
    fi
    if [ -n "${AGENT_PID}" ] && kill -0 "${AGENT_PID}" 2>/dev/null; then
        kill -TERM "${AGENT_PID}" 2>/dev/null || true
    fi
    pkill -TERM -f "gz sim" 2>/dev/null || true
    pkill -TERM -f "px4" 2>/dev/null || true
    wait 2>/dev/null || true
    exit 0
}

trap cleanup SIGTERM SIGINT

if [ "${ENABLE_MAVLINK_ROUTER}" = "true" ]; then
    mkdir -p /etc/mavlink-router 2>/dev/null || sudo mkdir -p /etc/mavlink-router 2>/dev/null || true
    sudo chown -R developer:developer /etc/mavlink-router 2>/dev/null || true

    if [ ! -f /etc/mavlink-router/main.conf ]; then
        cat <<EOF > /etc/mavlink-router/main.conf
[General]
TcpServerPort=${MAVLINK_TCP_PORT}
ReportStats=false

[UdpEndpoint flightcontroller]
Mode=Server
Address=127.0.0.1
Port=14550

[UdpEndpoint qgc]
Mode=Normal
Address=${MAVLINK_QGC_IP}
Port=${MAVLINK_QGC_PORT}
EOF
    fi

    echo "Starting mavlink-routerd..."
    mavlink-routerd -c /etc/mavlink-router/main.conf &
    MAVLINK_PID=$!
fi

echo "Starting MicroXRCEAgent on UDP port 8888..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!

cd /home/developer/PX4-Autopilot

echo "Starting PX4 SITL (${PX4_SIM_MODEL})..."
HEADLESS=1 make px4_sitl "${PX4_SIM_MODEL}" < /dev/stdin &
PX4_PID=$!

wait -n ${PX4_PID} ${AGENT_PID} ${MAVLINK_PID} 2>/dev/null || true
cleanup
