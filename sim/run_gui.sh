#!/bin/bash
set -e

# Directory of this script (pickme_drone/sim)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS_DIR="$SCRIPT_DIR/models"

echo "=== Starting Gazebo Harmonic GUI for PickmeDrone ==="

# 1. Find simulation container (supports both 'sim' and 'px4-sim')
CONTAINER_ID=$(docker compose -f "$PROJECT_DIR/compose.sim.yaml" ps -q sim 2>/dev/null || true)
if [ -z "$CONTAINER_ID" ]; then
    CONTAINER_ID=$(docker compose -f "$PROJECT_DIR/compose.sim.yaml" ps -q px4-sim 2>/dev/null || true)
fi

if [ -z "$CONTAINER_ID" ]; then
    echo "[ERROR] Simulation container ('sim' or 'px4-sim') is not running."
    echo "        Please start the simulation first with: make sim (or docker compose -f compose.sim.yaml up)"
    exit 1
fi

# 2. Automatically sync 3D models if not present locally
if [ ! -d "$MODELS_DIR" ] || [ -z "$(ls -A "$MODELS_DIR" 2>/dev/null)" ]; then
    echo "[INFO] 3D models not found locally in $MODELS_DIR."
    echo "[INFO] Copying drone models and meshes from px4-sim container..."
    mkdir -p "$MODELS_DIR"
    docker cp "$CONTAINER_ID:/home/developer/PX4-Autopilot/Tools/simulation/gz/models/." "$MODELS_DIR/"
    echo "[INFO] 3D models copied successfully."
fi

# 3. Get Container IP from Docker/OrbStack
PX4_IP=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$CONTAINER_ID")

if [ -z "$PX4_IP" ]; then
    echo "[ERROR] Could not determine IP address for px4-sim container."
    exit 1
fi

echo "[INFO] Connected to PX4 Gazebo Server at IP: $PX4_IP"
echo "[INFO] Models path: $MODELS_DIR"

# 4. Launch Gazebo GUI
export GZ_PARTITION=sim
export GZ_RELAY="$PX4_IP"
export GZ_SIM_RESOURCE_PATH="$MODELS_DIR"

echo "[INFO] Launching gz sim -g..."
gz sim -g
