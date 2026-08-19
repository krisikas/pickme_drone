# Project Memory & Constraints

This file serves as the system memory for the AI agent working on this project. The agent MUST read this file at the start of any task in this workspace.

## Tech Stack
- **Development Host OS**: macOS (Apple Silicon ARM64 via OrbStack / Docker)
- **Target Embedded OS**: Ubuntu 22.04 LTS (ARM64) on Radxa Rock 5C
- **Middleware**: ROS 2 Humble (LTS)
- **Simulation**: 
  - **Server (Headless)**: Gazebo Harmonic (`gz sim -s -r`) running inside `sim` Docker container.
  - **GUI Client**: Gazebo Harmonic (`gz sim -g`) running natively on macOS host (`brew install gz-harmonic`), connecting via `GZ_RELAY` and `GZ_PARTITION=sim`.
- **Flight Controller**: PX4-Autopilot (v1.15+, SITL mode for sim, Matek H743/H735 for real hardware)
- **Bridges**: Micro-XRCE-DDS Agent v2.4.3 (PX4 ↔ ROS 2), `ros_gz_bridge` (Gazebo ↔ ROS 2)
- **Sensors**: RealSense D435 / OakD-Lite (Depth / Pointcloud / RGB)

## Multi-Tier Container Architecture

See `ARCHITECTURE.md` for full details and diagrams.

### Docker Image Hierarchy
- **`pickme-base`** (`docker/base.Dockerfile`): `FROM ros:humble`. Foundation with px4_msgs + pickme_interfaces compiled into /opt/ros/humble.
- **`pickme-px4-bridge`** (`docker/px4-bridge.Dockerfile`): `FROM ubuntu:22.04`. Micro-XRCE-DDS Agent only. No ROS. Hardware: `/dev/ttyACM0`.
- **`pickme-camera`** (`docker/camera.Dockerfile`): `FROM pickme-base`. RealSense SDK + realsense-ros. Hardware: `/dev/bus/usb`.
- **`pickme-core`** (`docker/core.Dockerfile`): `FROM pickme-base`. Stable background services: `pickme_commander`. Future: SLAM, EKF, TF.
- **`pickme-dev`** (`docker/dev.Dockerfile`): `FROM pickme-base`. Dev sandbox. `src/pickme_missions` mounted via volume.

### Code Distribution Rules
- `pickme_interfaces` → compiled inside `pickme-base` (COPY + colcon build)
- `pickme_commander` → compiled inside `pickme-core` (COPY + colcon build)
- `pickme_missions` → NOT copied. Mounted into `pickme-dev` via volumes. Compiled manually inside container.

### Simulation vs Real
- Simulation: `compose/compose.sim.yaml` — `sim` + `sim-perception` replace `px4-bridge` + `camera`. `core` and `dev` are identical.
- Real drone: `compose/compose.real.yaml` — `px4-bridge` + `camera` + `core` + `dev`.

## Networking
- **Simulation**: Docker bridge network `ros_net`. Gazebo Transport uses `GZ_PARTITION=sim`. Host GUI connects via `GZ_RELAY=<container_ip>`.
- **Real drone**: `network_mode: host` for all containers. ROS 2 DDS discovery works automatically.

## Repository Layout
- `compose/`: Compose files (`compose/compose.sim.yaml`, `compose/compose.real.yaml`).
- `docker/`: Dockerfiles and container entrypoints (`docker/sim-entrypoint.sh`).
- `sim/`: Gazebo GUI launch script (`sim/run_gui.sh`), custom worlds and models.
- `src/`: ROS 2 packages (`pickme_commander`, `pickme_interfaces`, `pickme_missions`).

## Legacy
- `archipelago_container/` — old monolithic container from an earlier iteration of the project. Not part of the current architecture. Kept for reference.

## Agent Operational Rules
1. **Never write or edit code without user permission** when the user requests confirmation.
2. **PX4 SITL Execution**: Always launch simulation natively via `HEADLESS=1 make px4_sitl <model>`. Never manually start detached `gz sim` processes or use `PX4_GZ_STANDALONE=1`, as it breaks timesync and Gazebo sensor plugins.
3. **Gazebo Versioning**: Always use **Gazebo Harmonic** (`gz-harmonic`) conventions and APIs (`gz ...`). Do not use legacy Ignition Fortress (`ign`) or EOL Gazebo Garden.
4. **Container Discipline**: Never install simulation tools (`ros_gz`, Gazebo) into `base`, `core`, `dev`, or `camera` Dockerfiles. Simulation dependencies belong only in `sim.Dockerfile` and `sim-perception.Dockerfile`.
5. **Base Image**: Always use `ros:humble` (not `osrf/ros:humble-desktop`) as the foundation. It supports both AMD64 and ARM64.
6. **No docker commit**: All dependencies must be declared in Dockerfiles. Never use `docker commit` to persist changes.
7. **Verification**: After modifying Dockerfiles or compose configurations, verify with `docker compose -f compose/compose.sim.yaml config` and `make build`.
8. **No Placeholders**: Never use `// TODO` or placeholder implementations. Implement complete, functioning code.
