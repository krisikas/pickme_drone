# Project Memory & Constraints

This file serves as the system memory for the AI agent working on this project. The agent MUST read this file at the start of any task in this workspace.

## Tech Stack
- **Development Host OS**: macOS (Apple Silicon ARM64 via OrbStack / Docker)
- **Target Embedded OS**: Ubuntu 22.04 LTS (ARM64) on Radxa Rock 5C
- **Middleware**: ROS 2 Humble (LTS)
- **Simulation**: 
  - **Server (Headless)**: Gazebo Harmonic (`gz sim -s -r`) running inside `sim` Docker container (`docker/sim.Dockerfile`).
  - **GUI Client**: Gazebo Harmonic (`gz sim -g`) running natively on macOS host (`brew install gz-harmonic`), connecting via `GZ_RELAY` and `GZ_PARTITION=sim`.
- **Flight Controller**: PX4-Autopilot (v1.15.4+, SITL mode for sim, Matek H743/H735 for real hardware)
- **Bridges**: 
  - Micro-XRCE-DDS Agent v2.4.3 (PX4 ↔ ROS 2 DDS)
  - `ros_gz_bridge` via `parameter_bridge` with YAML config (Gazebo Harmonic ↔ ROS 2)
- **Sensors**: RealSense D435 / OakD-Lite (Depth / Pointcloud / RGB)

---

## Multi-Tier Container Architecture

See `ARCHITECTURE.md` for full details and diagrams.

### Docker Image Hierarchy
- **`pickme-base`** (`docker/base.Dockerfile`): `FROM ros:humble`. Foundation with `px4_msgs` (v1.15) + `pickme_interfaces` compiled into `/opt/ros/humble`.
- **`pickme-px4-bridge`** (`docker/px4-bridge.Dockerfile`): `FROM ubuntu:22.04`. Micro-XRCE-DDS Agent only. No ROS. Hardware: `/dev/ttyACM0`.
- **`pickme-camera`** (`docker/camera.Dockerfile`): `FROM pickme-base`. RealSense SDK + `realsense-ros`. Hardware: `/dev/bus/usb`.
- **`pickme-core`** (`docker/core.Dockerfile`): `FROM pickme-base`. Stable background services: `pickme_commander`. Future: SLAM, EKF, TF.
- **`pickme-dev`** (`docker/dev.Dockerfile`): `FROM pickme-base`. Dev sandbox. `src/pickme_missions` mounted via volume.
- **`pickme-sim`** (`docker/sim.Dockerfile`): `FROM ubuntu:22.04`. Headless PX4 SITL + Gazebo Harmonic Server + Micro-XRCE-DDS + MAVLink Router.
- **`pickme-sim-perception`** (`docker/sim-perception.Dockerfile`): `FROM ros:humble`. `ros_gz_bridge` parameter bridge mapping Gazebo topics to standard ROS 2 perception topics.

### Code Distribution Rules
- `pickme_interfaces` → compiled inside `pickme-base` (`COPY` + `colcon build`)
- `pickme_commander` → compiled inside `pickme-core` (`COPY` + `colcon build`)
- `pickme_missions` → NOT copied. Mounted into `pickme-dev` via volumes. Compiled manually inside container.

### Simulation vs Real
- **Simulation**: `compose/compose.sim.yaml` — `sim` + `sim-perception` replace `px4-bridge` + `camera`. `core` and `dev` are identical.
- **Real drone**: `compose/compose.real.yaml` — `px4-bridge` + `camera` + `core` + `dev`.

---

## Sensor & Perception Topic Contract

Both `pickme-camera` (real hardware) and `pickme-sim-perception` (simulation) MUST adhere to the following unified ROS 2 topic interface:

| Topic Name | Message Type | Purpose / Content | Simulation Source (Gazebo) |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock synchronization | `/clock` (`gz.msgs.Clock`) |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB Color stream | `/camera` (`gz.msgs.Image`) |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB Camera intrinsics | `/camera/camera_info` (`gz.msgs.CameraInfo`) |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Rectified depth map (float32 / meters) | `/depth_camera` (`gz.msgs.Image`) |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth Camera intrinsics | `/depth_camera/camera_info` (`gz.msgs.CameraInfo`) |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | 3D Point Cloud | `/depth_camera/points` (`gz.msgs.PointCloudPacked`) |

---

## QoS Guidelines & Flight Controller Communication

1. **PX4 Topics (`/fmu/in/*`, `/fmu/out/*`)**:
   - Must use `PX4_QOS`:
     - Reliability: `BEST_EFFORT`
     - Durability: `TRANSIENT_LOCAL`
     - History: `KEEP_LAST`, Depth: `1`
2. **Sensor Streams (`/camera/*`)**:
   - Use `SensorDataQoS` (`BEST_EFFORT`, `VOLATILE`, depth: 5) for high-bandwidth images and pointclouds.
3. **Offboard Heartbeat**:
   - `OffboardControlMode` messages must be published continuously at **>= 2 Hz** (standard: 10 Hz) before and during offboard flight, otherwise PX4 failsafe kicks in.

---

## Networking
- **Simulation**: Docker bridge network `ros_net`. Gazebo Transport uses `GZ_PARTITION=sim`. Host GUI connects via `GZ_RELAY=<container_ip>`.
- **Real drone**: `network_mode: host` for all containers. ROS 2 DDS discovery works automatically.

## Repository Layout
- `compose/`: Compose files (`compose/compose.sim.yaml`, `compose/compose.real.yaml`).
- `docker/`: Dockerfiles and container entrypoints (`docker/sim-entrypoint.sh`).
- `sim/`: Gazebo GUI launch script (`sim/run_gui.sh`), bridge config (`sim/config/bridge.yaml`), custom worlds and models.
- `src/`: ROS 2 packages (`pickme_bringup`, `pickme_commander`, `pickme_interfaces`, `pickme_missions`).

---

## Agent Operational Rules
1. **Never write or edit code without user permission** when the user requests confirmation.
2. **PX4 SITL Execution**: Always launch simulation natively via `HEADLESS=1 make px4_sitl <model>`. Never manually start detached `gz sim` processes or use `PX4_GZ_STANDALONE=1`, as it breaks timesync and Gazebo sensor plugins.
3. **Gazebo Versioning**: Always use **Gazebo Harmonic** (`gz-harmonic`) conventions and APIs (`gz ...`). Do not use legacy Ignition Fortress (`ign`) or EOL Gazebo Garden.
4. **Container Discipline**: Never install simulation tools (`ros_gz`, Gazebo) into `base`, `core`, `dev`, or `camera` Dockerfiles. Simulation dependencies belong only in `sim.Dockerfile` and `sim-perception.Dockerfile`.
5. **Base Image**: Always use `ros:humble` (not `osrf/ros:humble-desktop`) as the foundation. It supports both AMD64 and ARM64.
6. **No docker commit**: All dependencies must be declared in Dockerfiles. Never use `docker commit` to persist changes.
7. **Verification**: After modifying Dockerfiles or compose configurations, verify with `docker compose -f compose/compose.sim.yaml config` and `make build`.
8. **No Placeholders**: Never use `// TODO` or placeholder implementations. Implement complete, functioning code.
9. **Interface Isolation**: Custom messages/services must live exclusively in `pickme_interfaces`.
