#!/bin/bash
set -e

# Directory of this script (pickme_drone/sim)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS_DIR="$SCRIPT_DIR/models"

echo "=== Starting Gazebo Harmonic GUI for PickmeDrone ==="

# 1. Find simulation container (supports 'sim', 'pickme-sim', 'px4-sim')
CONTAINER_ID=$(docker compose -f "$PROJECT_DIR/compose/compose.sim.yaml" ps -q sim 2>/dev/null || docker compose -f "$PROJECT_DIR/compose.sim.yaml" ps -q sim 2>/dev/null || true)
if [ -z "$CONTAINER_ID" ]; then
    CONTAINER_ID=$(docker ps -q -f name=pickme-sim 2>/dev/null || true)
fi

if [ -z "$CONTAINER_ID" ]; then
    echo "[ERROR] Simulation container ('sim' / 'pickme-sim') is not running."
    echo "        Please start the simulation first with: make sim"
    exit 1
fi

# 2. Automatically sync 3D models if not present locally
if [ ! -d "$MODELS_DIR" ] || [ -z "$(ls -A "$MODELS_DIR" 2>/dev/null)" ]; then
    echo "[INFO] 3D models not found locally in $MODELS_DIR."
    echo "[INFO] Copying drone models and meshes from sim container..."
    mkdir -p "$MODELS_DIR"
    docker cp "$CONTAINER_ID:/home/developer/PX4-Autopilot/Tools/simulation/gz/models/." "$MODELS_DIR/"
    echo "[INFO] 3D models copied successfully."
fi

# 3. Get Container IP from Docker/OrbStack
PX4_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$CONTAINER_ID" | tr -d '[:space:]')

if [ -z "$PX4_IP" ]; then
    echo "[ERROR] Could not determine IP address for sim container."
    exit 1
fi

echo "[INFO] Connected to PX4 Gazebo Server at IP: $PX4_IP"
echo "[INFO] Models path: $MODELS_DIR"
echo "[INFO] GZ_PARTITION: sim"
echo "[INFO] GZ_RELAY: $PX4_IP"

# 4. Launch Gazebo GUI
# GZ_IP must be set to the Mac's IP on the interface facing the container network
# so that gz-transport binds the reply socket on the right interface.
# Without it, the service REP socket may bind to loopback (127.0.0.1)
# while the container sends replies to the Mac's OrbStack IP — causing EACCES.
HOST_FACING_IP=$(ifconfig | awk '/inet /{print $2}' | grep -v "127.0.0.1" | head -n 1)
echo "[INFO] GZ_IP (Mac host): $HOST_FACING_IP"
export GZ_PARTITION=sim
export GZ_RELAY="$PX4_IP"
export GZ_IP="$HOST_FACING_IP"
export GZ_SIM_RESOURCE_PATH="$MODELS_DIR"

echo "[INFO] Launching gz sim -g -v 4..."
exec gz sim -g -v 4 "$@"
