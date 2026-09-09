#!/bin/bash
set -e

FCU_DEVICE="${FCU_DEVICE:-/dev/ttyS0}"
FCU_BAUD="${FCU_BAUD:-921600}"

ENABLE_MAVLINK_ROUTER="${ENABLE_MAVLINK_ROUTER:-false}"
MAVLINK_DEVICE="${MAVLINK_DEVICE:-/dev/ttyACM0}"
MAVLINK_BAUD="${MAVLINK_BAUD:-921600}"
MAVLINK_TCP_PORT="${MAVLINK_TCP_PORT:-5760}"
MAVLINK_QGC_IP="${MAVLINK_QGC_IP:-10.42.0.255}"
MAVLINK_QGC_PORT="${MAVLINK_QGC_PORT:-14550}"

MAVLINK_PID=""
AGENT_PID=""

cleanup() {
    echo "Stopping FCU bridge processes..."
    if [ -n "${MAVLINK_PID}" ] && kill -0 "${MAVLINK_PID}" 2>/dev/null; then
        kill -TERM "${MAVLINK_PID}" 2>/dev/null || true
    fi
    if [ -n "${AGENT_PID}" ] && kill -0 "${AGENT_PID}" 2>/dev/null; then
        kill -TERM "${AGENT_PID}" 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    exit 0
}

trap cleanup SIGTERM SIGINT

if [ "${ENABLE_MAVLINK_ROUTER}" = "true" ]; then
    if [ -e "${MAVLINK_DEVICE}" ]; then
        chmod 666 "${MAVLINK_DEVICE}" 2>/dev/null || true
        mkdir -p /etc/mavlink-router
        
        if [ ! -f /etc/mavlink-router/main.conf ]; then
            cat <<EOF > /etc/mavlink-router/main.conf
[General]
TcpServerPort=${MAVLINK_TCP_PORT}
ReportStats=false

[UartEndpoint flightcontroller]
Device=${MAVLINK_DEVICE}
Baud=${MAVLINK_BAUD}

[UdpEndpoint qgc]
Mode=Normal
Address=${MAVLINK_QGC_IP}
Port=${MAVLINK_QGC_PORT}
EOF
        fi
        
        echo "Starting mavlink-routerd..."
        mavlink-routerd -c /etc/mavlink-router/main.conf &
        MAVLINK_PID=$!
    else
        echo "Warning: MAVLink device '${MAVLINK_DEVICE}' not found, skipping mavlink-router."
        echo "Available serial devices:"
        ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    fi
fi

if [ ! -e "${FCU_DEVICE}" ]; then
    echo "Warning: Serial device '${FCU_DEVICE}' not found!"
    echo "Available serial ports on system:"
    ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    
    # Auto-detection fallback for Radxa hardware UART and USB
    if [ -e "/dev/ttyS4" ]; then
        echo "--> Auto-fallback: Switching FCU_DEVICE to detected Radxa hardware UART /dev/ttyS4"
        FCU_DEVICE="/dev/ttyS4"
    elif [ -e "/dev/ttyACM0" ] && [ "${ENABLE_MAVLINK_ROUTER}" != "true" ]; then
        echo "--> Auto-fallback: Switching FCU_DEVICE to detected USB serial /dev/ttyACM0"
        FCU_DEVICE="/dev/ttyACM0"
    else
        echo "ERROR: No suitable serial device found for MicroXRCEAgent!"
        exit 1
    fi
fi

chmod 666 "${FCU_DEVICE}" 2>/dev/null || true

echo "Starting MicroXRCEAgent on ${FCU_DEVICE} at ${FCU_BAUD} baud..."
MicroXRCEAgent serial --dev "${FCU_DEVICE}" -b "${FCU_BAUD}" &
AGENT_PID=$!

wait -n ${AGENT_PID} ${MAVLINK_PID} 2>/dev/null || true
cleanup
