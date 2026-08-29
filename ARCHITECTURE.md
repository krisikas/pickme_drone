# PickmeDrone System Architecture

<div align="center">

**Engineering Architecture Specification & Technical Design Document**  
*ROS 2 Humble • PX4 Autopilot v1.15 • Gazebo Harmonic • Micro-XRCE-DDS • RTAB-Map SLAM*

</div>

---

## 1. Architectural Philosophy & Design Principles

The PickmeDrone system is built around a **Sim-First, Multi-Tier Decoupled Architecture**. It is engineered to provide high reliability, real-time deterministic control, and seamless transfer between virtual simulation (Gazebo Harmonic SITL) and physical embedded hardware (Radxa Rock 5C / Matek H743).

```text
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                  CORE ARCHITECTURAL TENETS                                       │
├──────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 1. Flight Core Fault Isolation   │ Perception crashes, web telemetry disconnections, or mission  │
│                                  │ bugs NEVER compromise the flight control loop (pickme-core).  │
├──────────────────────────────────┼───────────────────────────────────────────────────────────────┤
│ 2. Single-Node Offboard Loop     │ Heartbeat (10 Hz) and Trajectory Setpoints (50 Hz) execute    │
│                                  │ synchronously inside one C++ node to eliminate DDS jitter.    │
├──────────────────────────────────┼───────────────────────────────────────────────────────────────┤
│ 3. Sim-First Parity (Sim2Real)   │ 100% identical ROS 2 API contract between simulated and       │
│                                  │ physical drone hardware.                                      │
├──────────────────────────────────┼───────────────────────────────────────────────────────────────┤
│ 4. Strict Interface Segregation  │ All custom ROS 2 messages, services, and actions live solely  │
│                                  │ in pickme_interfaces, avoiding cyclic build dependencies.     │
├──────────────────────────────────┼───────────────────────────────────────────────────────────────┤
│ 5. Universal Coordinate Geometry │ Full compliance with ROS REP-105 (ENU/FLU) and REP-103, with  │
│                                  │ automatic runtime TF2 resolution for arbitrary goal frames.   │
└──────────────────────────────────┴───────────────────────────────────────────────────────────────┘
```

---

## 2. Multi-Tier Container Hierarchy (Sim-First)

The system leverages Docker layer caching and multi-stage container specialization to separate build dependencies, runtime environments, and compute workloads:


```text
                                ros:humble (Official Base Image)
                                                │
                                                ▼
                                 ┌──────────────────────────────┐
                                 │         pickme-base          │
                                 │ ──────────────────────────── │
                                 │ • ROS 2 Humble Desktop       │
                                 │ • px4_msgs (v1.15 overlay)   │
                                 │ • pickme_interfaces overlay  │
                                 │ • Eigen3, TF2, colcon tools  │
                                 └──────────────┬───────────────┘
                                                │
                 ┌──────────────────────────────┼──────────────────────────────┬──────────────────────────────┐
                 │ FROM pickme-base             │ FROM pickme-base             │ FROM pickme-base             │ FROM pickme-base
                 ▼                              ▼                              ▼                              ▼
  ┌──────────────────────────────┐ ┌──────────────────────────────┐ ┌──────────────────────────────┐ ┌──────────────────────────────┐
  │         pickme-core          │ │      pickme-perception       │ │       pickme-foxglove        │ │          pickme-dev          │
  │ ──────────────────────────── │ │ ──────────────────────────── │ │ ──────────────────────────── │ │ ──────────────────────────── │
  │ • pickme_offboard (50Hz)     │ │ • RTAB-Map RGB-D Odometry    │ │ • foxglove_bridge WebSocket  │ │ • Interactive dev sandbox    │
  │ • pickme_odometry (TF2 Broad)│ │ • RTAB-Map 3D SLAM / OctoMap │ │ • Port 8765 (UI Telemetry)   │ │ • GDB, RQT, Teleop tools     │
  │ • pickme_bringup (Core)      │ │ • False loop-closure filter  │ │ • Zero impact on flight core │ │ • Host workspace volume mount│
  └──────────────────────────────┘ └──────────────────────────────┘ └──────────────────────────────┘ └──────────────────────────────┘

  ┌──────────────────────────────┐ ┌──────────────────────────────┐
  │         pickme-sim           │ │    pickme-sim-perception     │
  │ ──────────────────────────── │ │ ──────────────────────────── │
  │ • Ubuntu 22.04 + gz-harmonic │ │ • ros_gz_bridge              │
  │ • PX4 SITL v1.15.4 (Headless)│ │ • Parameter bridge (YAML)    │
  │ • Micro-XRCE-DDS Agent (8888)│ │ • Converts Gazebo RGB/Depth  │
  │ • MAVLink Router (TCP 5760)  │ │   and PointCloud to ROS 2    │
  └──────────────────────────────┘ └──────────────────────────────┘
```

### Container Responsibility Matrix

| Container | Purpose | Key Daemons & Nodes | Exposed Ports / Mounts |
|---|---|---|---|
| **`pickme-base`** | Common base image | System libraries, compiled message definitions | None (Intermediate build layer) |
| **`pickme-sim`** | Physics & Autopilot Backend | `gz sim -s -r`, `px4`, `MicroXRCEAgent -p 8888`, `mavlink-routerd` | `5760/tcp` (MAVLink QGC), FastDDS XML, World/Model mounts |
| **`pickme-sim-perception`** | Simulation Sensor Bridge | `ros_gz_bridge parameter_bridge` | `bridge.yaml` config mount |
| **`pickme-core`** | Mission-Critical Flight Control | `odometry_node`, `offboard_node` | FastDDS XML, shared ROS domain (`ROS_DOMAIN_ID=0`) |
| **`pickme-perception`** | GPS-Denied Localization & Mapping | `rgbd_odometry`, `rtabmap` SLAM | FastDDS XML |
| **`pickme-foxglove`** | Web Dashboard Bridge | `foxglove_bridge` WebSocket daemon | `8765/tcp` (WebSocket streaming) |
| **`pickme-dev`** | User Code Execution Sandbox | Developer shell, Python SDK, testing tools | `../workspace` $\to$ `/home/developer/workspace` |

---

## 3. Communication Topologies & Protocols

PickmeDrone combines three distinct high-performance communication protocols tailored to their specific bandwidth and determinism requirements:

```text
┌───────────────────────────┐         gz-transport          ┌───────────────────────────┐
│        pickme-sim         │ ────────────────────────────► │   pickme-sim-perception   │
│  (Gazebo Harmonic Server) │   (RGB, Depth, Points, Clock) │      (ros_gz_bridge)      │
└─────────────┬─────────────┘                               └─────────────┬─────────────┘
              │                                                           │
              │ Micro-XRCE-DDS (UDP 8888)                                 │ ROS 2 Sensor Topics
              │ • /fmu/out/vehicle_odometry                               │ • /camera/color/image_raw
              │ • /fmu/in/trajectory_setpoint                             │ • /camera/depth/image_rect_raw
              │ • /fmu/in/offboard_control_mode                           │ • /camera/depth/color/points
              │ • /fmu/in/vehicle_command                                 │ • /clock
              ▼                                                           ▼
┌───────────────────────────────────────────────────────────────────────────────────────┐
│                                   ROS 2 Humble FastDDS                                │
│                                                                                       │
│   ┌───────────────────────────┐   /visual_odometry    ┌───────────────────────────┐   │
│   │        pickme-core        │ ◄──────────────────── │     pickme-perception     │   │
│   │  (OdometryNode, Offboard) │                       │  (RTAB-Map Visual Odom)   │   │
│   └─────────────┬─────────────┘                       └───────────────────────────┘   │
│                 │                                                                     │
│                 │ ROS 2 Services & Telemetry                                          │
│                 ├───────────────────────────────────────┐                             │
│                 ▼                                       ▼                             │
│   ┌───────────────────────────┐           ┌───────────────────────────┐               │
│   │        pickme-dev         │           │      pickme-foxglove      │               │
│   │ (User Missions / Python)  │           │   (foxglove_bridge WS)    │               │
│   └───────────────────────────┘           └─────────────┬─────────────┘               │
└─────────────────────────────────────────────────────────┼─────────────────────────────┘
                                                          │ WebSocket (ws://localhost:8765)
                                                          ▼
                                            ┌───────────────────────────┐
                                            │      Foxglove Studio      │
                                            └───────────────────────────┘
```

### 3.1 Micro-XRCE-DDS (PX4 ↔ ROS 2 Core)
* **Agent Port:** UDP `8888`
* **QoS Profile:**
  * Reliability: `BEST_EFFORT`
  * Durability: `TRANSIENT_LOCAL`
  * History: `KEEP_LAST`, Depth: `1`
* **High-Rate Streams:**
  * Offboard Control Mode: `10 Hz` (`/fmu/in/offboard_control_mode`)
  * Trajectory Setpoint: `50 Hz` (`/fmu/in/trajectory_setpoint`)
  * Vehicle Odometry (EKF2 Output): `50 Hz` (`/fmu/out/vehicle_odometry`)
  * Visual Odometry (EV Input to EKF2): `20-30 Hz` (`/fmu/in/vehicle_visual_odometry`)

### 3.2 Gazebo Transport & `ros_gz_bridge`
* **Transport Partition:** `GZ_PARTITION=sim`
* **Relay Mechanism:** The native macOS GUI connects to the simulation container via `GZ_RELAY=<container_ip>` and `GZ_IP=<host_ip>`.
* **Sensor Topics:** Low-latency ZeroMQ-based shared memory pipelines inside the simulation container translate raw camera sensor buffers directly into standard ROS 2 `sensor_msgs` without IPC copy penalties.

---

## 4. ROS 2 Package Architecture (`src/`)

```text
src/
├── pickme_interfaces/                 # [DATA CONTRACTS] Custom ROS 2 interfaces
│   ├── msg/
│   │   └── FlightStatus.msg           # Comprehensive state machine, telemetry & diagnostics
│   ├── srv/
│   │   ├── Navigate.srv               # 3D Position, Yaw, Speed, Frame ID, Auto-Arm
│   │   ├── SetAltitude.srv            # Independent Z altitude control
│   │   ├── SetYaw.srv                 # Independent heading control
│   │   └── GetTelemetry.srv           # Full vehicle state, velocities, and diagnostics
│   └── action/
│       └── FollowTrajectory.action    # Continuous multi-waypoint path following
│
├── pickme_odometry/                   # [STATE ESTIMATION & TF2 BROADCASTING]
│   ├── include/pickme_odometry/
│   │   └── odometry_node.hpp          # OdometryNode definition
│   ├── src/
│   │   └── odometry_node.cpp          # PX4 NED -> ROS ENU transforms, dynamic /tf,
│   │                                  # OakD-Lite static TF tree, VO -> PX4 bridge
│   └── launch/
│       └── odometry.launch.py
│
├── pickme_offboard/                   # [REAL-TIME FLIGHT CONTROL CORE]
│   ├── include/pickme_offboard/
│   │   ├── heartbeat_manager.hpp      # 10 Hz OffboardControlMode pulse generator
│   │   ├── command_sender.hpp         # VehicleCommand dispatcher & telemetry feedback
│   │   ├── trajectory_tracker.hpp     # 50 Hz closed-loop setpoint interpolator & lag limiter
│   │   ├── safety_guard.hpp           # 3D Geofence, NaN check, odometry watchdog
│   │   └── offboard_node.hpp          # Orchestrator, 20Hz /pickme/flight_status & fast services
│   ├── src/
│   │   ├── heartbeat_manager.cpp
│   │   ├── command_sender.cpp
│   │   ├── trajectory_tracker.cpp
│   │   ├── safety_guard.cpp
│   │   └── offboard_node.cpp
│   └── launch/
│       └── offboard.launch.py
│
└── pickme_bringup/                    # [ORCHESTRATION & CONFIGURATION]
    ├── launch/
    │   ├── core.launch.py             # Launches pickme_odometry + pickme_offboard
    │   ├── perception.launch.py       # Launches RTAB-Map RGB-D VO & 3D SLAM
    │   └── foxglove.launch.py         # Launches foxglove_bridge server
    └── config/
        ├── core_params.yaml           # Centralized flight dynamics, geofence, and frame parameters
        └── perception.yaml            # RTAB-Map feature extraction and graph optimizer parameters
```

---

## 5. Coordinate Frames & Mathematics (REP-105 / REP-103)

Aerial robotics systems deal with two fundamentally conflicting coordinate representations:
1. **Aeronautical Standard (PX4):** North-East-Down (NED) world frame with Front-Right-Down (FRD) body frame.
2. **Robotics Standard (ROS 2 REP-105):** East-North-Up (ENU) world frame with Forward-Left-Up (FLU) body frame.

PickmeDrone implements exact, closed-form geometric and quaternion transformations inside `pickme_odometry` and `pickme_offboard`.

```text
                             map (Global Earth-Fixed Frame)
                              │
                              ▼
                             odom (Continuous World Frame — ENU)
                              │
                              ▼ [Dynamic /tf from PX4 VehicleOdometry]
                             base_link (Drone Body Center of Mass — FLU)
                              │
               ┌──────────────┴──────────────┬─────────────────────────────┐
               ▼                             ▼                             ▼
              body                       imu_link                     camera_link
       (Yaw-only Leveled)             (Colocated IMU)            (Physical Camera Center)
                                                                           │
                                              ┌────────────────────────────┴───────────────────────────┐
                                              ▼                                                        ▼
                                     camera_depth_frame                                       camera_color_frame
                                      (FLU Center)                                         (FLU Offset: Y = -15mm)
                                              │                                                        │
                                              ▼                                                        ▼
                                 camera_depth_optical_frame                               camera_color_optical_frame
                                  (RDF: Optical Z Forward)                                 (RDF: Optical Z Forward)
```

### 5.1 Coordinate Translation
$$\begin{bmatrix} x_{\text{enu}} \\ y_{\text{enu}} \\ z_{\text{enu}} \end{bmatrix} = \begin{bmatrix} y_{\text{ned}} \\ x_{\text{ned}} \\ -z_{\text{ned}} \end{bmatrix},\qquad \begin{bmatrix} x_{\text{ned}} \\ y_{\text{ned}} \\ z_{\text{ned}} \end{bmatrix} = \begin{bmatrix} y_{\text{enu}} \\ x_{\text{enu}} \\ -z_{\text{enu}} \end{bmatrix}$$

### 5.2 Quaternion Orientation Conversion
Given a PX4 orientation quaternion in NED/FRD $q_{\text{px4}} = \begin{bmatrix} w_{\text{px4}} & x_{\text{px4}} & y_{\text{px4}} & z_{\text{px4}} \end{bmatrix}^T$, the corresponding ROS ENU/FLU quaternion $q_{\text{ros}} = \begin{bmatrix} x_{\text{ros}} & y_{\text{ros}} & z_{\text{ros}} & w_{\text{ros}} \end{bmatrix}^T$ is computed as:

$$w_{\text{ros}} = \frac{\sqrt{2}}{2} (w_{\text{px4}} + z_{\text{px4}})$$
$$x_{\text{ros}} = \frac{\sqrt{2}}{2} (x_{\text{px4}} + y_{\text{px4}})$$
$$y_{\text{ros}} = \frac{\sqrt{2}}{2} (x_{\text{px4}} - y_{\text{px4}})$$
$$z_{\text{ros}} = \frac{\sqrt{2}}{2} (w_{\text{px4}} - z_{\text{px4}})$$

---

## 6. Offboard Flight Dynamics & Control Mechanics

```text
                                        Navigate Request (x, y, z, yaw, speed, frame_id)
                                                              │
                                                              ▼
                                               ┌──────────────────────────────┐
                                               │   TF2 Universal Resolver     │
                                               │ (Converts frame_id -> odom)  │
                                               └──────────────┬───────────────┘
                                                              │
                                                              ▼
                                               ┌──────────────────────────────┐
                                               │         Safety Guard         │
                                               │ • 3D Geofence Verification   │
                                               │ • NaN / Infinity Sanitization│
                                               │ • Odometry Freshness Guard   │
                                               └──────────────┬───────────────┘
                                                              │ Goal Accepted
                                                              ▼
                                               ┌──────────────────────────────┐
                                               │      Trajectory Tracker      │
                                               │ • Velocity bounded step:     │
                                               │   ΔS = V_target * Δt         │
                                               │ • Broadcast navigate_target  │
                                               └──────────────┬───────────────┘
                                                              │
                                                              ├────────────────────────────────────────┐
                                                              ▼ 50 Hz Setpoint Stream                  ▼ 10 Hz Heartbeat
                                               ┌──────────────────────────────┐         ┌──────────────────────────────┐
                                               │  /fmu/in/trajectory_setpoint │         │ /fmu/in/offboard_control_mode│
                                               └──────────────┬───────────────┘         └──────────────┬───────────────┘
                                                              │                                        │
                                                              ▼                                        ▼
                                               ┌───────────────────────────────────────────────────────────────┐
                                               │                PX4 Flight Management Unit (FMU)               │
                                               │ • Position / Velocity Controller                              │
                                               │ • Rate Controller & Motor Allocation                          │
                                               └───────────────────────────────────────────────────────────────┘
```

### 6.1 Why Setpoint Tracking is Interpolated in ROS 2
1. **Dynamic Task Speed:** In native PX4 offboard position mode, the drone flies to a setpoint at a fixed global parameter speed (`MPC_XY_CRUISE`). In PickmeDrone, `TrajectoryTracker` computes incremental setpoint steps at 50 Hz ($\Delta t = 20\text{ ms}$):
   $$\vec{S}_{k+1} = \vec{S}_k + \min\left(\|\vec{S}_{\text{goal}} - \vec{S}_k\|,\, v_{\text{target}} \cdot \Delta t\right) \cdot \frac{\vec{S}_{\text{goal}} - \vec{S}_k}{\|\vec{S}_{\text{goal}} - \vec{S}_k\|}$$
   This allows missions to command precise velocities (e.g., $0.2\text{ m/s}$ for precision sensor scanning vs $2.5\text{ m/s}$ for rapid transit).
2. **Elimination of Motor Saturation:** Sudden large position steps cause maximum acceleration demands that can induce ESC brownouts or pitch/roll instability. Smooth setpoint propagation produces gentle, bounded motor response.

### 6.2 Why Offboard Classes Share a Single C++ Node
* **Deterministic Synchronicity:** PX4 failsafes require that the Offboard Heartbeat (10 Hz) and Setpoints (50 Hz) are continuously fed without phase lag. Splitting them into separate processes introduces DDS inter-process scheduling jitter.
* **Zero-Copy State Validation:** `SafetyGuard`, `TrajectoryTracker`, and `CommandSender` evaluate vehicle state in the same thread cycle without message deserialization overhead.

---

## 7. GPS-Denied Perception & Visual SLAM

In indoor and GPS-denied environments, the drone achieves autonomous localization by fusing high-rate **RGB-D Visual Odometry** with PX4's 24-state **Extended Kalman Filter (EKF2)**.

```text
 ┌──────────────────────┐   /camera/color/image_raw (RGB)
 │  RealSense D435 /    │   /camera/depth/image_rect_raw (Depth)
 │  OakD-Lite (Gazebo)  │────────────────────────────────────────┐
 └──────────────────────┘                                        │
                                                                 ▼
                                                 ┌──────────────────────────────┐
                                                 │     rtabmap_odom Node        │
                                                 │ • ORB Feature Extractor      │
                                                 │ • Frame-to-Map Matcher       │
                                                 │ • Optical flow motion guess  │
                                                 └──────────────┬───────────────┘
                                                                │ /visual_odometry (nav_msgs/Odometry)
                                                                ▼
                                                 ┌──────────────────────────────┐
                                                 │     OdometryNode Bridge      │
                                                 │ (Converts ROS ENU -> PX4 NED)│
                                                 └──────────────┬───────────────┘
                                                                │ /fmu/in/vehicle_visual_odometry
                                                                ▼
                                                 ┌──────────────────────────────┐
                                                 │       PX4 EKF2 Estimator     │
                                                 │ • Fuses IMU + Baro + VO      │
                                                 │ • EKF2_EV_CTRL = 15          │
                                                 │ • EKF2_GPS_CTRL = 0          │
                                                 └──────────────┬───────────────┘
                                                                │ /fmu/out/vehicle_odometry
                                                                ▼
                                                 ┌──────────────────────────────┐
                                                 │     Dynamic TF Broadcaster   │
                                                 │ • Broadcasts odom->base_link │
                                                 │ • Publishes /odom topic      │
                                                 └──────────────────────────────┘
```

### 7.1 RTAB-Map Configuration Parameters
* **Feature Detector:** ORB (`Vis/FeatureType: 8`, 400 features/frame).
* **Anti-Drift Filter:** Rejects floor noise with `Vis/MinInliers: 20` and `RGBD/OptimizeMaxError: 1.5m`.
* **3D Occupancy Grid:** Ray-traced 3D voxel grid (`Grid/3D: true`, 5 cm voxel resolution) detecting obstacles from $0.3\text{ m}$ to $5.0\text{ m}$ in real time.

---

## 8. Simulation-to-Reality (Sim2Real) Transfer

PickmeDrone was designed from day one to deploy to physical hardware without changing a single line of flight or mission code:

| Component | Simulation (Docker macOS/Linux) | Physical Drone (Radxa Rock 5C SBC) |
|---|---|---|
| **Flight Controller** | PX4 SITL (`gz_x500_depth`) | Matek H743-SLIM / Pixhawk 6C (UART Telem) |
| **DDS Transport** | Micro-XRCE-DDS UDP localhost | Micro-XRCE-DDS `/dev/ttyS0` @ 921600 baud |
| **Depth Sensor** | Gazebo Harmonic `gz-sensors` DepthCamera | Physical RealSense D435 / OakD-Lite via USB 3.0 |
| **Perception Bridge** | `pickme-sim-perception` (`ros_gz_bridge`) | Native `realsense2_camera` driver |
| **Flight Core** | `pickme-core` (`core.launch.py`) | `pickme-core` (`core.launch.py`) — **100% Identical** |
| **Autonomous Missions** | `pickme-dev` (`example_mission.py`) | `pickme-dev` (`example_mission.py`) — **100% Identical** |

