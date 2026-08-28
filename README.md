# PickmeDrone: Autonomous Aerial Robotics Platform

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-343434.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.15.4-blue.svg?logo=drone&logoColor=white)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Docker](https://img.shields.io/badge/Docker-Multi--Container-2496ED.svg?logo=docker&logoColor=white)](https://www.docker.com/)
[![C++17](https://img.shields.io/badge/C++-17-00599C.svg?logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)
[![Micro-XRCE-DDS](https://img.shields.io/badge/DDS-Micro--XRCE--DDS_v2.4-green.svg)](https://micro-xrce-dds.docs.eprosima.com/)
[![RTAB-Map](https://img.shields.io/badge/SLAM-RTAB--Map_RGB--D-purple.svg)](http://introlab.github.io/rtabmap/)
[![Foxglove](https://img.shields.io/badge/Visualization-Foxglove_Studio-FF4081.svg?logo=foxglove&logoColor=white)](https://foxglove.dev/)

**A production-grade, Sim-First autonomous drone platform designed for GPS-denied navigation, 3D visual SLAM, offboard flight control, and seamless Simulation-to-Reality (Sim2Real) deployment.**

[Key Features](#key-features) •
[System Architecture](#system-architecture) •
[Prerequisites](#prerequisites) •
[Quick Start](#quick-start) •
[Makefile Reference](#makefile-reference) •
[ROS 2 Service API](#ros-2-service-api-reference) •
[Mission Development](#mission-development) •
[Perception & SLAM](#perception--slam-pipeline) •
[Troubleshooting](#troubleshooting--faq)

</div>

---

## Overview

PickmeDrone is an end-to-end aerial robotics development framework built on **ROS 2 Humble**, **PX4-Autopilot (v1.15)**, and **Gazebo Harmonic**. It provides a modular, decoupled architecture that isolates critical flight control services from perception pipelines and user mission code.

Developed with a **Sim-First** philosophy, the platform allows developers to build, simulate, benchmark, and deploy autonomous navigation missions entirely inside containerized environments on macOS (Apple Silicon / Intel) and Linux, transferring directly to physical embedded companion computers (e.g., Radxa Rock 5C, NVIDIA Jetson) with zero code modifications.

---

## Key Features

* **Decoupled Multi-Container Architecture:** Full fault isolation between PX4 SITL simulation (`sim`), perception and SLAM (`perception`), stable flight core (`core`), Foxglove WebSocket telemetry (`foxglove`), and developer sandboxes (`dev`).
* **High-Frequency Micro-XRCE-DDS Bridge:** Direct 50 Hz trajectory setpoint streaming and 10 Hz offboard heartbeat communication with the PX4 Flight Management Unit (FMU) over low-latency DDS.
* **REP-105 & REP-103 Coordinate Frame Standards:** Built-in dynamic and static TF2 broadcasting converting aerospace NED/FRD frames into standard robotics ENU/FLU conventions, complete with camera optical frame transformations.
* **Smooth Trajectory Tracking & Waypoint Interpolation:** Continuous velocity-bounded setpoint generator ($\Delta \vec{S} = \vec{v}_{\text{target}} \cdot \Delta t$) preventing motor saturation, abrupt accelerations, and setpoint jumps.
* **Universal Frame Navigation:** Issue navigation waypoints in any coordinate frame (`odom`, `map`, `body`, `base_link`, `camera_link`). The offboard controller automatically resolves frames via TF2.
* **GPS-Denied Visual Odometry & 3D SLAM:** Out-of-the-box RTAB-Map RGB-D visual odometry fused into PX4 EKF2 External Vision (`EKF2_EV_CTRL`), paired with 3D voxel occupancy grid mapping and loop-closure verification.
* **Real-Time Safety Guard:** Active 3D geofencing, NaN/Inf floating-point sanity checking, maximum velocity clamping, and odometry freshness watchdogs.
* **Native macOS 3D GUI & Web Dashboards:** Headless simulation server paired with a native Gazebo 3D GUI client via `GZ_RELAY`, alongside real-time web telemetry via Foxglove Studio on port `8765`.

---

## System Architecture (Sim)

The environment runs across isolated Docker containers connected through a high-speed bridge network (`ros_net`), with a lightweight native 3D visualizer running on the host machine:

```text
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                          HOST MACHINE (macOS / Linux)                                    │
│                                                                                                          │
│   ┌──────────────────────────────────────────────┐        ┌──────────────────────────────────────────┐   │
│   │        Gazebo Harmonic 3D GUI (Native)       │        │             Foxglove Studio              │   │
│   │         gz sim -g (via GZ_RELAY)             │        │          ws://localhost:8765             │   │
│   └──────────────────────▲───────────────────────┘        └────────────────────▲─────────────────────┘   │
│                          │ GZ Transport                                        │ WebSocket               │
│   ┌──────────────────────┼─────────────────────────────────────────────────────┼─────────────────────┐   │
│   │                      │                Docker Network (`ros_net`)           │                     │   │
│   │                      │                                                     │                     │   │
│   │   ┌──────────────────┴───────────┐         ┌────────────────────────┐      │                     │   │
│   │   │          pickme-sim          │         │ pickme-sim-perception  │      │                     │   │
│   │   │ ──────────────────────────── │ gz-pipe │ ────────────────────── │      │                     │   │
│   │   │ • Gazebo Server (gz sim -s)  │────────►│ • ros_gz_bridge        │      │                     │   │
│   │   │ • PX4 SITL (x500_depth)      │         │   (RGB, Depth, Point-  │      │                     │   │
│   │   │ • Micro-XRCE-DDS Agent       │         │    cloud, /clock)      │      │                     │   │
│   │   └──────────────┬───────────────┘         └───────────┬────────────┘      │                     │   │
│   │                  │ XRCE-DDS (port 8888)                │ ROS 2 Topics      │                     │   │
│   │                  │                                     │                   │                     │   │
│   │                  ▼                                     ▼                   │                     │   │
│   │   ┌──────────────────────────────┐         ┌────────────────────────┐      │                     │   │
│   │   │         pickme-core          │         │   pickme-perception    │      │                     │   │
│   │   │ ──────────────────────────── │◄────────│ ────────────────────── │      │                     │   │
│   │   │ • pickme_offboard (50Hz Ctrl)│ /visual │ • RTAB-Map RGB-D Odom  │      │                     │   │
│   │   │ • pickme_odometry (TF2 Broad)│ odometry│ • 3D OctoMap / SLAM    │      │                     │   │
│   │   │ • Fast Services API          │         │ • Loop Closure Guard   │      │                     │   │
│   │   └──────────────┬───────────────┘         └────────────────────────┘      │                     │   │
│   │                  │ ROS 2 Services & TF2                                    │                     │   │
│   │                  ├─────────────────────────────────────────────────────────┤                     │   │
│   │                  │                                                         │                     │   │
│   │                  ▼                                                         ▼                     │   │
│   │   ┌──────────────────────────────┐                         ┌────────────────────────┐            │   │
│   │   │          pickme-dev          │                         │    pickme-foxglove     │            │   │
│   │   │ ──────────────────────────── │                         │ ────────────────────── │            │   │
│   │   │ • Mission Execution & Python │                         │ • foxglove_bridge      │            |   │
│   │   │ • Development Workspace      │                         │ • Isolated Telemetry   │            |   │
│   │   │ • GDB, RQT, CLI Diagnostics  │                         └────────────────────────┘            |   │
│   │   └──────────────────────────────┘                                                               |   │
│   └──────────────────────────────────────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### Container Responsibilities

| Container | Base Image | Key Components | Role & Responsibilities |
|---|---|---|---|
| **`pickme-base`** | `ros:humble` | `px4_msgs`, `pickme_interfaces`, Eigen3, TF2 | Foundation image containing shared interfaces and system dependencies. |
| **`pickme-sim`** | `ubuntu:22.04` | PX4 SITL v1.15.4, Gazebo Harmonic (`gz-sim`), Micro-XRCE-DDS Agent | Runs physics, aerodynamics, motor simulation, and FMU autopilot logic in headless mode. |
| **`pickme-sim-perception`** | `osrf/ros:humble-desktop` | `ros_gz_bridge`, `parameter_bridge` | Bridges simulated camera streams (RGB, Depth, PointCloud) and simulation `/clock` to ROS 2 topics. |
| **`pickme-core`** | `pickme-base` | `pickme_offboard`, `pickme_odometry`, `pickme_bringup` | Mission-critical flight core. Maintains 10 Hz offboard heartbeat, 50 Hz trajectory tracker, TF2 tree, and safety guards. |
| **`pickme-perception`** | `pickme-base` | `rtabmap_odom`, `rtabmap_slam`, `g2o` | Computes visual odometry (`/visual_odometry`) and builds 3D occupancy maps in GPS-denied environments. |
| **`pickme-foxglove`** | `pickme-base` | `foxglove_bridge` | Dedicated WebSocket server (port `8765`) for browser/desktop telemetry visualization. |
| **`pickme-dev`** | `pickme-base` | Interactive tools, Python SDK, workspace volume | Sandboxed workspace for compiling user nodes, running autonomous mission scripts, and debugging. |

---

## Prerequisites

### 1. Host Operating System
* **macOS:** Apple Silicon (M1/M2/M3/M4) or Intel with **OrbStack** (recommended for performance) or **Docker Desktop**.
* **Linux:** Ubuntu 22.04 LTS (x86_64 or aarch64) with **Docker Engine** & **Docker Compose v2**.

### 2. Native Gazebo Harmonic (for macOS Host GUI)
To run the high-performance native 3D visualizer on macOS without X11 or VNC latency:
```bash
brew tap osrf/simulation
brew install gz-harmonic
```

---

## Quick Start

### Step 1: Build Docker Images
Build the base interface layer and all downstream container services:
```bash
cd pickme_drone
make build
```

### Step 2: Launch the Simulation Backend
Start the complete stack (PX4 SITL, Gazebo Server, DDS Bridges, Flight Core, Perception, and Foxglove):
```bash
make sim
```
*Description:* PX4 SITL boots, connects to Gazebo Harmonic Server, establishes Micro-XRCE-DDS at UDP `8888`, and `pickme-core` initializes the 10 Hz heartbeat and 50 Hz trajectory controller.

### Step 3: Launch Native 3D Visualization
Open a new terminal tab and start the native Gazebo GUI:
```bash
make gui
```
*Description:* The launch script detects the simulation container IP, syncs required 3D meshes (OakD-Lite, x500 frame, textures), and attaches the native Gazebo GUI to the running simulation partition.

### Step 4: Open Telemetry Dashboard (Optional)
1. Open [Foxglove Studio](https://foxglove.dev/) (Desktop App or Web Browser).
2. Connect to: `ws://localhost:8765`.
3. Visualize 3D TF transforms, odometry poses, camera streams (`/camera/color/image_raw`), point clouds (`/camera/depth/color/points`), and RTAB-Map 3D grid maps.

---

## Makefile Reference

The root `Makefile` provides streamlined commands for the entire lifecycle:

| Target | Description |
|---|---|
| `make build` | Builds `pickme-base` and compiles all simulation container images. |
| `make build-base` | Rebuilds only `pickme-base` (use after modifying `pickme_interfaces`). |
| `make sim` | Starts the complete simulation stack in the foreground. |
| `make gui` | Connects native Gazebo Harmonic GUI on macOS host to the running simulation. |
| `make down` | Gracefully stops and removes all active simulation containers and networks. |
| `make logs` | Streams aggregated real-time logs from all running containers. |
| `make gz-topics` | Queries active Gazebo Transport topics inside the simulation partition. |
| `make shell-dev` | Enters an interactive bash shell in the `pickme-dev` developer container. |
| `make shell-core` | Enters an interactive bash shell in the `pickme-core` flight controller container. |
| `make shell-sim` | Enters an interactive bash shell in the `pickme-sim` PX4/Gazebo container. |
| `make shell-perception` | Enters an interactive bash shell in the `pickme-perception` SLAM container. |
| `make shell-px4` | Attaches directly to the interactive PX4 NuttX/POSIX shell console. |

---

## ROS 2 Service API Reference

`pickme_offboard` exposes high-level, non-blocking asynchronous ROS 2 services (<1 ms response time) under the `/pickme` namespace:

| Service | Type | Description | Example CLI Call |
|---|---|---|---|
| `/pickme/arm` | `std_srvs/srv/Trigger` | Arms the vehicle motors. | `ros2 service call /pickme/arm std_srvs/srv/Trigger {}` |
| `/pickme/disarm` | `std_srvs/srv/Trigger` | Disarms motors and cancels active trajectory. | `ros2 service call /pickme/disarm std_srvs/srv/Trigger {}` |
| `/pickme/takeoff` | `std_srvs/srv/Trigger` | Arms, engages offboard mode, and climbs to default altitude. | `ros2 service call /pickme/takeoff std_srvs/srv/Trigger {}` |
| `/pickme/land` | `std_srvs/srv/Trigger` | Dispatches autonomous landing command to PX4. | `ros2 service call /pickme/land std_srvs/srv/Trigger {}` |
| `/pickme/navigate` | `pickme_interfaces/srv/Navigate` | Navigates to $(x, y, z, \text{yaw})$ at specific speed in any frame. | `ros2 service call /pickme/navigate pickme_interfaces/srv/Navigate "{x: 2.0, y: 1.0, z: 2.0, yaw: 0.0, speed: 0.8, frame_id: 'odom', auto_arm: false}"` |
| `/pickme/set_altitude` | `pickme_interfaces/srv/SetAltitude` | Adjusts altitude while preserving horizontal coordinates. | `ros2 service call /pickme/set_altitude pickme_interfaces/srv/SetAltitude "{z: 3.5, frame_id: 'odom'}"` |
| `/pickme/set_yaw` | `pickme_interfaces/srv/SetYaw` | Rotates heading to desired angle (radians). | `ros2 service call /pickme/set_yaw pickme_interfaces/srv/SetYaw "{yaw: 1.57, frame_id: 'odom'}"` |
| `/pickme/get_telemetry` | `pickme_interfaces/srv/GetTelemetry` | Returns real-time state, pose, velocity, and battery status. | `ros2 service call /pickme/get_telemetry pickme_interfaces/srv/GetTelemetry "{frame_id: 'odom'}"` |

### Field Details for `/pickme/navigate`
* `x`, `y`, `z` *(float64)*: Target coordinates in meters.
* `yaw` *(float64)*: Target heading in radians ($[-\pi, \pi]$).
* `speed` *(float32)*: Linear flight speed in m/s (defaults to `default_speed` if set to 0).
* `frame_id` *(string)*: Coordinate frame for target:
  * `'odom'` / `'map'`: Global world-fixed coordinates.
  * `'body'`: Relative to current drone heading and position ($+X$ forward, $+Y$ left, $+Z$ up).
  * `'base_link'`: Relative to full 3D body orientation.
  * `'camera_link'`: Relative to camera optical mount.
* `auto_arm` *(bool)*: Automatically arms and switches to Offboard mode if true.

---

## Mission Development

User missions execute inside the `pickme-dev` sandbox and communicate with `pickme-core` exclusively via standard ROS 2 services and topics.

### Example Mission Script (`workspace/example_mission.py`)

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pickme_interfaces.srv import Navigate

class AutonomousSquareMission(Node):
    def __init__(self):
        super().__init__('square_mission')
        self.nav_client = self.create_client(Navigate, '/pickme/navigate')
        self.takeoff_client = self.create_client(Trigger, '/pickme/takeoff')
        self.land_client = self.create_client(Trigger, '/pickme/land')

        self.get_logger().info('Waiting for PickmeDrone flight services...')
        self.nav_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()

    def send_goal(self, x, y, z, yaw, speed=0.8, frame_id='odom'):
        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        req.frame_id = frame_id
        req.auto_arm = False
        return self.nav_client.call_async(req)

    def run(self):
        # 1. Takeoff to 2.0 meters
        self.get_logger().info('Executing Takeoff...')
        self.takeoff_client.call_async(Trigger.Request())
        rclpy.spin_once(self, timeout_sec=5.0)

        # 2. Fly relative 2m forward in body frame
        self.get_logger().info('Flying 2m forward relative to body...')
        self.send_goal(x=2.0, y=0.0, z=0.0, yaw=0.0, frame_id='body')
```

### Running Your Mission
```bash
# 1. Enter the dev container
make shell-dev

# 2. Run the mission script
python3 workspace/example_mission.py
```

---

## Coordinate Frames & TF2 Architecture

The platform strictly follows **REP-105** (Coordinate Frames for Mobile Platforms) and **REP-103** (Standard Units of Measure).

```text
map (Global World Origin — Earth-fixed)
 └── odom (Continuous World Frame — East-North-Up ENU)
      └── base_link (Drone Body Center of Mass — Forward-Left-Up FLU)
           ├── body (Leveled Horizontal Body Frame — Yaw only)
           ├── camera_link (Mount Center — FLU)
           │    ├── camera_depth_frame (FLU)
           │    │    └── camera_depth_optical_frame (Right-Down-Forward RDF)
           │    ├── camera_color_frame (FLU, Y = -15mm offset)
           │    │    └── camera_color_optical_frame (RDF)
           │    └── camera_optical_frame (RDF Alias)
           └── imu_link (Colocated with Flight Controller)
```

### Mathematical Coordinate Conversions
* **Position:** $\begin{bmatrix} x_{\text{enu}} \\ y_{\text{enu}} \\ z_{\text{enu}} \end{bmatrix} = \begin{bmatrix} y_{\text{ned}} \\ x_{\text{ned}} \\ -z_{\text{ned}} \end{bmatrix}$
* **Orientation (Quaternion):**
  $$w_{\text{ros}} = \frac{\sqrt{2}}{2}(w_{\text{px4}} + z_{\text{px4}}),\quad x_{\text{ros}} = \frac{\sqrt{2}}{2}(x_{\text{px4}} + y_{\text{px4}})$$
  $$y_{\text{ros}} = \frac{\sqrt{2}}{2}(x_{\text{px4}} - y_{\text{px4}}),\quad z_{\text{ros}} = \frac{\sqrt{2}}{2}(w_{\text{px4}} - z_{\text{px4}})$$

---

## Perception & SLAM Pipeline

Both simulated depth cameras (`OakD-Lite` / `RealSense D435`) and physical hardware adhere to the standardized topic interface:

| Topic Name | Message Type | Description |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock synchronization. |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB Color stream (1280x720 @ 30 FPS). |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB camera intrinsic calibration parameters. |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Rectified depth map (Float32 in meters). |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | 3D RGB point cloud. |
| `/visual_odometry` | `nav_msgs/msg/Odometry` | Visual odometry computed by RTAB-Map. |
| `/odom` | `nav_msgs/msg/Odometry` | Fused state odometry from PX4 EKF2 in ENU coordinates. |

### Visual SLAM Configuration
The perception stack runs **RTAB-Map RGB-D Odometry** with:
* **ORB Feature Matching** (`Vis/FeatureType: 8`, 400 features per frame).
* **Frame-to-Map Motion Estimation** for smooth tracking over feature-sparse terrains.
* **Strict False Loop-Closure Rejection** (`Vis/MinInliers: 20`, `RGBD/OptimizeMaxError: 1.5m`) preventing drift on repetitive carpet/tile textures.
* **Direct EKF2 Fusion** (`EKF2_EV_CTRL = 15`) sending `/visual_odometry` into PX4's EKF2 over `/fmu/in/vehicle_visual_odometry`.

---

## Troubleshooting & FAQ

### 1. Gazebo GUI shows a black screen or missing mesh warnings
* **Cause:** 3D meshes (`.dae`, `.sdf`, textures) must be available locally on the host.
* **Resolution:** The `make gui` script synchronizes models automatically. To force manual synchronization:
  ```bash
  mkdir -p sim/models
  docker cp $(docker compose -f compose/compose.sim.yaml ps -q sim):/home/developer/PX4-Autopilot/Tools/simulation/gz/models/. ./sim/models/
  ```

### 2. Offboard mode rejects engagement or triggers Failsafe Land
* **Cause:** Heartbeat signal frequency dropped below 2 Hz or state odometry timed out.
* **Resolution:** Inspect `make logs` to verify that `pickme-core` actively streams `/fmu/in/offboard_control_mode` (10 Hz) and `TrajectorySetpoint` (50 Hz). Verify that `use_sim_time: true` is configured across all active nodes.

### 3. Procedure for clean shutdown
* Send `Ctrl+C` in the terminal executing `make sim`, or execute `make down` in any terminal to terminate all services and tear down networks.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).
Distributed under standard open-source terms for research and commercial robotics applications.

