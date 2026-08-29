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

## Multi-Tier Container Architecture (Sim-First)

See `ARCHITECTURE.md` for full details and diagrams.

### Docker Image Hierarchy
- **`pickme-base`** (`docker/base.Dockerfile`): `FROM ros:humble`. Foundation with `px4_msgs` (v1.15) + `pickme_interfaces` compiled into `/opt/ros/humble`. (Strictly interfaces only).
- **`pickme-sim`** (`docker/sim.Dockerfile`): `FROM ubuntu:22.04`. Headless PX4 SITL + Gazebo Harmonic Server + Micro-XRCE-DDS + MAVLink Router.
- **`pickme-sim-perception`** (`docker/sim-perception.Dockerfile`): `FROM osrf/ros:humble-desktop`. `ros_gz_bridge` mapping Gazebo topics to standard ROS 2 perception topics.
- **`pickme-core`** (`docker/core.Dockerfile`): `FROM pickme-base`. Stable flight background services (`pickme_offboard`, `pickme_odometry`, `pickme_bringup`).
- **`pickme-perception`** (`docker/perception.Dockerfile`): `FROM pickme-base`. RTAB-Map Visual Odometry and 3D SLAM.
- **`pickme-dev`** (`docker/dev.Dockerfile`): `FROM pickme-base`. Dev sandbox with tools (`gdb`, `rqt`, `tmux`, `teleop`). Workspace mounted via volume.

### Code Distribution Rules
- `pickme_interfaces` → compiled inside `pickme-base` (`COPY` + `colcon build`)
- `pickme_offboard`, `pickme_odometry`, `pickme_bringup` → compiled inside `pickme-core` (`COPY` + `colcon build`)
- `pickme_bringup` (perception launch/config) → compiled inside `pickme-perception` (`COPY` + `colcon build`)
- `pickme_missions` → NOT copied. Mounted into `pickme-dev` via volumes. Compiled manually inside container.

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
- `src/`: ROS 2 packages (`pickme_bringup`, `pickme_odometry`, `pickme_offboard`, `pickme_interfaces`).

---

## Coordinate Frames & TF2 Architecture

The project strictly follows **REP-105** (Coordinate Frames for Mobile Platforms) and **REP-103** (Standard Units of Measure & Coordinate Conventions).

### PX4 ↔ ROS 2 Conversion (NED/FRD → ENU/FLU)
- **World Frame**: PX4 uses NED (North-East-Down). ROS 2 uses **ENU** (East-North-Up).
  $$x_{enu} = y_{ned},\quad y_{enu} = x_{ned},\quad z_{enu} = -z_{ned}$$
- **Body Frame**: PX4 uses FRD (Front-Right-Down). ROS 2 uses **FLU** (Forward-Left-Up).
- **Quaternion Conversion** ($q_{px4} = [w, x, y, z] \to q_{ros} = (x, y, z, w)$):
  $$w_{ros} = \frac{\sqrt{2}}{2} (w_{px4} + z_{px4}),\quad x_{ros} = \frac{\sqrt{2}}{2} (x_{px4} + y_{px4})$$
  $$y_{ros} = \frac{\sqrt{2}}{2} (x_{px4} - y_{px4}),\quad z_{ros} = \frac{\sqrt{2}}{2} (w_{px4} - z_{px4})$$

### Complete TF Tree Topology (OakD-Lite on x500 Standard)
```text
odom (ENU: World-fixed continuous odometry)
 │
 └── base_link (FLU: Drone body center) [Dynamic /tf from PX4 VehicleOdometry]
      │
      ├── camera_link (FLU: Center of OakD-Lite mount, X=+0.12m, Y=+0.03m, Z=+0.242m)
      │    │
      │    ├── camera_depth_frame (FLU: 0, 0, 0)
      │    │    └── camera_depth_optical_frame (RDF: Optical axis for depth & pointcloud)
      │    │
      │    ├── camera_color_frame (FLU: X=0, Y=-0.015m, Z=0 — RGB sensor physical offset)
      │    │    └── camera_color_optical_frame (RDF: Optical axis for RGB image)
      │    │
      │    └── camera_optical_frame (RDF: Generic optical alias pointing to depth optical)
      │
      └── imu_link (FLU: Colocated with drone center of mass)
```

### Dual-Mode TF Strategy (Simulation ↔ Real Drone)
- **Simulation (`publish_camera_tf: true`, default)**:
  `odometry_node` (in `pickme_odometry`) publishes the complete camera TF tree because Gazebo has no physical EEPROM calibration chip.
  *Note on Gazebo DepthCamera*: Gazebo Harmonic's `gz-sensors` generates 3D point cloud coordinates natively in the FLU robot body frame ($+X$ forward, $+Y$ left, $+Z$ up). Therefore, the simulated point cloud topic `/camera/depth/color/points` attaches directly to `camera_depth_frame` (FLU) in simulation to avoid double-rotating the points.
- **Real Drone (`publish_camera_tf: false`)**:
  `odometry_node` publishes only `odom -> base_link` and `base_link -> camera_link`. The internal camera frames (`camera_link -> ... -> optical_frames`) are broadcast directly by the official camera driver.

---

## Code Style & Formatting Rules

### 1. C++ Braces Rule (1TBS / K&R Style) — STRICT REQUIREMENT
**The opening curly brace `{` MUST ALWAYS be on the SAME line as the declaration/statement.** Never place `{` on a new line.

```cpp
// ✅ CORRECT (Brace on same line):
namespace pickme_core {

class CommanderNode : public rclcpp::Node {
public:
  CommanderNode() : Node("commander_node") {
    if (condition) {
      do_something();
    } else {
      do_other();
    }
  }

  void handle_arm() {
    // ...
  }
};

}  // namespace pickme_core

// ❌ WRONG (Never put brace on next line):
void handle_arm()
{
}
```

### 2. Comments Policy (Self-Documenting Code)
- **No excessive or obvious comments** inside method bodies. Code must be self-documenting with clear naming.
- **File Header Description**: A brief 2-4 line description block at the very top of each file explaining its purpose.
- **Section Dividers**: Use clean line labels to organize logical blocks (`// === Section Name ===`).

#### Comment Examples:
```cpp
// ✅ CORRECT (Clean header + Section dividers + Self-documenting code):
/**
 * TransformNode — Centralized coordinate frame (TF2) and odometry broadcaster.
 *
 * Converts PX4 aerospace coordinates (NED/FRD) to ROS 2 standard (ENU/FLU).
 * Broadcasts dynamic odom->base_link TF, RealSense static frames, and /odom topic.
 */

// === Parameter Declarations ===
odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");

// === Dynamic Transforms Callback ===
void handle_vehicle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  // ...
}

// ❌ WRONG (Excessive, redundant comments):
// create a variable x
int x = 5;
// check if x is greater than 0
if (x > 0) {
  // print log message to console
  RCLCPP_INFO(get_logger(), "x is positive");
}
```

### 3. Naming Conventions
- **C++ Files**: `snake_case.cpp`, `snake_case.hpp`
- **Classes & Structs**: `PascalCase` (`CommanderNode`, `TransformNode`)
- **Methods & Functions**: `snake_case()` (`handle_arm()`, `publish_static_transforms()`)
- **Member Variables**: `snake_case_` with trailing underscore (`offboard_mode_pub_`, `cam_x_`)
- **Constants**: `UPPER_SNAKE_CASE` or `kCamelCase` (`SQRT2_2`, `PI`)
- **ROS Entities**: Packages, nodes, topics, services, and TF frames always use `snake_case`.

---

## Agent Operational Rules
1. **Never write or edit code without user permission** when the user requests confirmation.
2. **PX4 SITL Execution**: Always launch simulation natively via `HEADLESS=1 make px4_sitl <model>`. Never manually start detached `gz sim` processes or use `PX4_GZ_STANDALONE=1`, as it breaks timesync and Gazebo sensor plugins.
3. **Gazebo Versioning**: Always use **Gazebo Harmonic** (`gz-harmonic`) conventions and APIs (`gz ...`). Do not use legacy Ignition Fortress (`ign`) or EOL Gazebo Garden.
4. **Container Discipline**: Never install simulation tools (`ros_gz`, Gazebo) into `base`, `core`, `dev`, or `camera` Dockerfiles. Simulation dependencies belong only in `sim.Dockerfile` and `sim-perception.Dockerfile`.
5. **Base Image**: Always use `ros:humble` (not `osrf/ros:humble-desktop`) as the foundation. It supports both AMD64 and ARM64. `osrf/ros:humble-desktop` only for sim-perception.
6. **No docker commit**: All dependencies must be declared in Dockerfiles. Never use `docker commit` to persist changes.
7. **Verification**: After modifying Dockerfiles or compose configurations, verify with `docker compose -f compose/compose.sim.yaml config` and `make build`.
8. **No Placeholders**: Never use `// TODO` or placeholder implementations. Implement complete, functioning code.
9. **Interface Isolation**: Custom messages/services must live exclusively in `pickme_interfaces`.
10. **C++ Code Style**: Always enforce 1TBS `{` placement on the same line and self-documenting comment rules.
