/**
 * ============================================================================
 * PickmeDrone — Offboard Flight Coordinator Node
 * ============================================================================
 *
 * Central flight management node for ROS 2 + PX4 Autopilot over Micro-XRCE-DDS.
 *
 * Features:
 * - Continuous 10 Hz Heartbeat & 50 Hz Closed-Loop Trajectory Setpoints
 * - Robust Flight State Machine (DISARMED, ARMED, TAKEOFF, NAVIGATING, HOVERING, LANDING)
 * - 20 Hz Telemetry & Flight Status Publisher (/pickme/flight_status)
 * - Fast non-blocking ROS 2 services (<1ms response time)
 * - Universal 3D TF2 target frame resolution (odom, map, body, base_link, camera_link)
 * - Real-time Safety Guard (Geofence, NaN/Inf checks, speed limits, odometry watchdog)
 * 
 * ----------------------------------------------------------------------------
 * CLI Usage Examples:
 * ----------------------------------------------------------------------------
 * 1. Telemetry inspection:
 *    ros2 service call /pickme/get_telemetry pickme_interfaces/srv/GetTelemetry "{frame_id: 'odom'}"
 * 
 * 2. Arm vehicle:
 *    ros2 service call /pickme/arm std_srvs/srv/Trigger {}
 * 
 * 3. Takeoff to default altitude (2.0m):
 *    ros2 service call /pickme/takeoff std_srvs/srv/Trigger {}
 * 
 * 4. Navigate in global world frame (odom):
 *    ros2 service call /pickme/navigate pickme_interfaces/srv/Navigate "{x: 2.0, y: 1.0, z: 2.0, yaw: 0.0, speed: 0.5, frame_id: 'odom', auto_arm: false}"
 * 
 * 5. Navigate relative to drone's horizontal heading (1 meter forward, 0.5m left):
 *    ros2 service call /pickme/navigate pickme_interfaces/srv/Navigate "{x: 1.0, y: 0.5, z: 0.0, yaw: 0.0, speed: 0.5, frame_id: 'body', auto_arm: false}"
 * 
 * 6. Change altitude or yaw independently:
 *    ros2 service call /pickme/set_altitude pickme_interfaces/srv/SetAltitude "{z: 3.0, frame_id: 'odom'}"
 *    ros2 service call /pickme/set_yaw pickme_interfaces/srv/SetYaw "{yaw: 1.57, frame_id: 'odom'}"
 * 
 * 7. Switch to Offboard flight mode:
 *    ros2 service call /pickme/offboard std_srvs/srv/Trigger {}
 * 
 * 8. Land vehicle:
 *    ros2 service call /pickme/land std_srvs/srv/Trigger {}
 * 
 * 9. Disarm vehicle:
 *    ros2 service call /pickme/disarm std_srvs/srv/Trigger {}
 * ============================================================================
 */

#include "pickme_offboard/offboard_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pickme_offboard {

constexpr double SQRT2_2 = 0.70710678118654752440;

OffboardNode::OffboardNode(const rclcpp::NodeOptions & options)
: Node("offboard_node", options) {
  // === Parameter Declarations ===
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "navigate_target");
  default_speed_ = this->declare_parameter<double>("default_speed", 1.0);
  max_speed_ = this->declare_parameter<double>("max_speed", 4.0);
  max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 1.5);
  takeoff_altitude_ = this->declare_parameter<double>("takeoff_altitude", 2.0);
  geofence_radius_ = this->declare_parameter<double>("geofence_radius", 25.0);
  geofence_min_alt_ = this->declare_parameter<double>("geofence_min_alt", 0.0);
  geofence_max_alt_ = this->declare_parameter<double>("geofence_max_alt", 15.0);
  odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 1.0);
  target_reached_threshold_ = this->declare_parameter<double>("target_reached_threshold", 0.15);

  // === Callback Groups for Concurrency & Deadlock Prevention ===
  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // === Initialize Modular Components ===
  heartbeat_mgr_ = std::make_unique<HeartbeatManager>(this);
  cmd_sender_ = std::make_unique<CommandSender>(this, sensor_cb_group_);
  safety_guard_ = std::make_unique<SafetyGuard>(
    geofence_radius_, geofence_min_alt_, geofence_max_alt_, max_speed_, max_yaw_rate_);
  trajectory_tracker_ = std::make_unique<TrajectoryTracker>(
    this, odom_frame_, target_frame_, default_speed_, target_reached_threshold_);

  // === TF2 Buffer & Listener ===
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // === Subscribers & Publishers ===
  rclcpp::SubscriptionOptions sensor_sub_opts;
  sensor_sub_opts.callback_group = sensor_cb_group_;

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
    std::bind(&OffboardNode::handle_odometry, this, std::placeholders::_1),
    sensor_sub_opts);

  flight_status_pub_ = this->create_publisher<pickme_interfaces::msg::FlightStatus>(
    "/pickme/flight_status", 10);

  // === 50 Hz Control Loop (20 ms) & 20 Hz Status Loop (50 ms) ===
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&OffboardNode::control_loop, this),
    timer_cb_group_);

  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&OffboardNode::status_publish_loop, this),
    timer_cb_group_);

  // === Service Servers (Reentrant Callback Group) ===
  arm_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/arm",
    std::bind(&OffboardNode::handle_arm, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  disarm_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/disarm",
    std::bind(&OffboardNode::handle_disarm, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  offboard_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/offboard",
    std::bind(&OffboardNode::handle_offboard, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  takeoff_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/takeoff",
    std::bind(&OffboardNode::handle_takeoff, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  land_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/land",
    std::bind(&OffboardNode::handle_land, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  navigate_service_ = this->create_service<pickme_interfaces::srv::Navigate>(
    "/pickme/navigate",
    std::bind(&OffboardNode::handle_navigate, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  set_altitude_service_ = this->create_service<pickme_interfaces::srv::SetAltitude>(
    "/pickme/set_altitude",
    std::bind(&OffboardNode::handle_set_altitude, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  set_yaw_service_ = this->create_service<pickme_interfaces::srv::SetYaw>(
    "/pickme/set_yaw",
    std::bind(&OffboardNode::handle_set_yaw, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  telemetry_service_ = this->create_service<pickme_interfaces::srv::GetTelemetry>(
    "/pickme/get_telemetry",
    std::bind(&OffboardNode::handle_get_telemetry, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  RCLCPP_INFO(this->get_logger(), "OffboardNode initialized (Modular, Concurrency-Safe & Status-Enabled)");
}

void OffboardNode::handle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  last_odom_time_ = this->get_clock()->now();

  // Convert PX4 NED -> ROS ENU
  current_pos_.x = static_cast<double>(msg->position[1]);
  current_pos_.y = static_cast<double>(msg->position[0]);
  current_pos_.z = -static_cast<double>(msg->position[2]);

  current_vel_.x = static_cast<double>(msg->velocity[1]);
  current_vel_.y = static_cast<double>(msg->velocity[0]);
  current_vel_.z = -static_cast<double>(msg->velocity[2]);

  // Orientation: NED/FRD -> ENU/FLU
  const double w_px4 = static_cast<double>(msg->q[0]);
  const double x_px4 = static_cast<double>(msg->q[1]);
  const double y_px4 = static_cast<double>(msg->q[2]);
  const double z_px4 = static_cast<double>(msg->q[3]);

  double w_ros = SQRT2_2 * (w_px4 + z_px4);
  double x_ros = SQRT2_2 * (x_px4 + y_px4);
  double y_ros = SQRT2_2 * (x_px4 - y_px4);
  double z_ros = SQRT2_2 * (w_px4 - z_px4);

  current_pos_.yaw = std::atan2(
    2.0 * (w_ros * z_ros + x_ros * y_ros),
    1.0 - 2.0 * (y_ros * y_ros + z_ros * z_ros));

  // Initialize trajectory tracker hold position on ground as soon as first odometry arrives
  if (!trajectory_tracker_->is_initialized()) {
    trajectory_tracker_->snap_to_position(current_pos_.x, current_pos_.y, current_pos_.z, current_pos_.yaw);
  }
}

void OffboardNode::control_loop() {
  auto now = this->get_clock()->now();

  // 1. Validate Odometry Freshness
  std::string odom_err;
  if (!safety_guard_->validate_odometry_freshness(now, last_odom_time_, odom_timeout_sec_, odom_err)) {
    last_error_ = odom_err;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Safety watchdog: %s", odom_err.c_str());
    return;
  }

  // 2. Update Flight State Machine transitions
  update_flight_state();

  // 3. Stream Closed-Loop Trajectory Setpoints (unless streaming is paused, e.g. during landing)
  constexpr double dt = 0.02;  // 50 Hz = 20ms
  trajectory_tracker_->update_and_publish(dt, now, current_pos_, current_vel_);
}

void OffboardNode::update_flight_state() {
  bool armed = cmd_sender_->is_armed();
  bool offboard = cmd_sender_->is_offboard();

  if (!armed) {
    flight_state_ = FlightState::DISARMED;
    return;
  }

  if (flight_state_ == FlightState::DISARMED && armed) {
    flight_state_ = offboard ? FlightState::HOVERING : FlightState::ARMED;
    return;
  }

  if (flight_state_ == FlightState::TAKEOFF) {
    if (trajectory_tracker_->is_target_reached()) {
      flight_state_ = FlightState::HOVERING;
      RCLCPP_INFO(this->get_logger(), "Takeoff complete. Transitioned to HOVERING.");
    }
  } else if (flight_state_ == FlightState::NAVIGATING) {
    if (trajectory_tracker_->is_target_reached()) {
      flight_state_ = FlightState::HOVERING;
      RCLCPP_INFO(this->get_logger(), "Target reached. Transitioned to HOVERING.");
    }
  } else if (flight_state_ == FlightState::LANDING) {
    if (!armed) {
      flight_state_ = FlightState::DISARMED;
      trajectory_tracker_->reset_target();
      RCLCPP_INFO(this->get_logger(), "Touchdown & Disarm detected. Landing complete.");
    }
  }
}

void OffboardNode::status_publish_loop() {
  auto now = this->get_clock()->now();
  pickme_interfaces::msg::FlightStatus status{};
  status.header.stamp = now;
  status.header.frame_id = odom_frame_;

  status.state = flight_state_to_str(flight_state_);
  status.connected = cmd_sender_->is_connected();
  status.armed = cmd_sender_->is_armed();
  status.offboard_mode = cmd_sender_->is_offboard();
  status.target_reached = trajectory_tracker_->is_target_reached();

  status.nav_state = cmd_sender_->get_nav_state();
  status.nav_state_str = cmd_sender_->get_nav_state_str();

  status.current_position.x = current_pos_.x;
  status.current_position.y = current_pos_.y;
  status.current_position.z = current_pos_.z;

  status.current_velocity.x = current_vel_.x;
  status.current_velocity.y = current_vel_.y;
  status.current_velocity.z = current_vel_.z;

  status.current_yaw = static_cast<float>(current_pos_.yaw);

  auto target = trajectory_tracker_->get_target_setpoint();
  status.target_position.x = target.x;
  status.target_position.y = target.y;
  status.target_position.z = target.z;
  status.target_yaw = static_cast<float>(target.yaw);
  status.target_speed = static_cast<float>(trajectory_tracker_->get_target_speed());
  status.remaining_distance = static_cast<float>(trajectory_tracker_->get_remaining_distance());

  status.battery_voltage = cmd_sender_->get_battery_voltage();
  status.battery_percentage = cmd_sender_->get_battery_remaining();
  status.last_error = last_error_;

  flight_status_pub_->publish(status);
}

bool OffboardNode::resolve_target_to_odom(
  const std::string & frame_id,
  double req_x, double req_y, double req_z, double req_yaw,
  double & out_x, double & out_y, double & out_z, double & out_yaw,
  std::string & err_msg) {
  // 1. Direct world frame case (odom only)
  if (frame_id.empty() || frame_id == odom_frame_) {
    out_x = req_x;
    out_y = req_y;
    out_z = req_z;
    out_yaw = req_yaw;
    return true;
  }

  // 2. Full 3D TF2 transformation (supports map, body, base_link, camera frames, etc.)
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      odom_frame_, frame_id, tf2::TimePointZero, tf2::durationFromSec(0.05));

    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header.frame_id = frame_id;
    pose_in.pose.position.x = req_x;
    pose_in.pose.position.y = req_y;
    pose_in.pose.position.z = req_z;

    tf2::Quaternion q_in;
    q_in.setRPY(0.0, 0.0, req_yaw);
    pose_in.pose.orientation.x = q_in.x();
    pose_in.pose.orientation.y = q_in.y();
    pose_in.pose.orientation.z = q_in.z();
    pose_in.pose.orientation.w = q_in.w();

    geometry_msgs::msg::PoseStamped pose_out;
    tf2::doTransform(pose_in, pose_out, tf_stamped);

    out_x = pose_out.pose.position.x;
    out_y = pose_out.pose.position.y;
    out_z = pose_out.pose.position.z;

    tf2::Quaternion q_out(
      pose_out.pose.orientation.x,
      pose_out.pose.orientation.y,
      pose_out.pose.orientation.z,
      pose_out.pose.orientation.w);
    tf2::Matrix3x3 m(q_out);
    double roll, pitch;
    m.getRPY(roll, pitch, out_yaw);

    // Normalize yaw to [-PI, PI]
    while (out_yaw > PI) out_yaw -= 2.0 * PI;
    while (out_yaw < -PI) out_yaw += 2.0 * PI;

    return true;
  } catch (const tf2::TransformException & ex) {
    err_msg = "TF2 transform failed from '" + frame_id + "' to '" + odom_frame_ + "': " + ex.what();
    return false;
  }
}

// === Service Handlers ===
void OffboardNode::handle_arm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "ARM requested (arming and activating Offboard stream)");

  if (!trajectory_tracker_->is_initialized()) {
    trajectory_tracker_->snap_to_position(current_pos_.x, current_pos_.y, current_pos_.z, current_pos_.yaw);
  }
  heartbeat_mgr_->set_position_control(true);
  heartbeat_mgr_->start();
  trajectory_tracker_->set_streaming_enabled(true);

  std::string arm_msg;
  bool arm_ok = cmd_sender_->arm(arm_msg);
  std::string mode_msg;
  bool mode_ok = cmd_sender_->set_offboard_mode(mode_msg);

  flight_state_ = FlightState::ARMED;
  response->success = arm_ok && mode_ok;
  response->message = arm_msg + " | " + mode_msg;
}

void OffboardNode::handle_disarm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "DISARM requested");
  std::string msg;
  bool success = cmd_sender_->disarm(msg);

  trajectory_tracker_->reset_target();
  flight_state_ = FlightState::DISARMED;

  response->success = success;
  response->message = msg;
}

void OffboardNode::handle_offboard(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "OFFBOARD mode requested");

  if (!trajectory_tracker_->is_initialized()) {
    trajectory_tracker_->snap_to_position(current_pos_.x, current_pos_.y, current_pos_.z, current_pos_.yaw);
  }
  heartbeat_mgr_->set_position_control(true);
  heartbeat_mgr_->start();
  trajectory_tracker_->set_streaming_enabled(true);

  std::string mode_msg;
  bool success = cmd_sender_->set_offboard_mode(mode_msg);

  response->success = success;
  response->message = mode_msg;
}

void OffboardNode::handle_takeoff(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  auto now = this->get_clock()->now();
  std::string odom_err;
  if (!safety_guard_->validate_odometry_freshness(now, last_odom_time_, odom_timeout_sec_, odom_err)) {
    response->success = false;
    response->message = "Takeoff rejected: " + odom_err;
    last_error_ = response->message;
    return;
  }

  if (flight_state_ == FlightState::NAVIGATING || flight_state_ == FlightState::TAKEOFF) {
    response->success = false;
    response->message = "Takeoff rejected: vehicle already airborne in state " + flight_state_to_str(flight_state_);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "TAKEOFF requested to altitude %.1fm", takeoff_altitude_);

  if (!trajectory_tracker_->is_initialized()) {
    trajectory_tracker_->snap_to_position(current_pos_.x, current_pos_.y, current_pos_.z, current_pos_.yaw);
  }
  heartbeat_mgr_->set_position_control(true);
  heartbeat_mgr_->start();
  trajectory_tracker_->set_streaming_enabled(true);

  std::string arm_msg;
  cmd_sender_->arm(arm_msg);
  std::string mode_msg;
  cmd_sender_->set_offboard_mode(mode_msg);

  double target_z = current_pos_.z + takeoff_altitude_;
  if (target_z > geofence_max_alt_) {
    target_z = geofence_max_alt_;
  }
  trajectory_tracker_->set_target(
    current_pos_.x, current_pos_.y, target_z, current_pos_.yaw, default_speed_);

  flight_state_ = FlightState::TAKEOFF;
  response->success = true;
  response->message = "Takeoff initiated (Target Alt: " + std::to_string(target_z) + "m)";
}

void OffboardNode::handle_land(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "LAND requested");

  trajectory_tracker_->set_streaming_enabled(false);
  heartbeat_mgr_->set_position_control(false);
  trajectory_tracker_->reset_target();

  std::string land_msg;
  bool success = cmd_sender_->land(land_msg);
  flight_state_ = FlightState::LANDING;

  response->success = success;
  response->message = land_msg;
}

void OffboardNode::handle_navigate(
  const std::shared_ptr<pickme_interfaces::srv::Navigate::Request> request,
  std::shared_ptr<pickme_interfaces::srv::Navigate::Response> response) {
  auto now = this->get_clock()->now();
  std::string odom_err;
  if (!safety_guard_->validate_odometry_freshness(now, last_odom_time_, odom_timeout_sec_, odom_err)) {
    response->success = false;
    response->message = "Navigate rejected: " + odom_err;
    last_error_ = response->message;
    return;
  }

  double target_x, target_y, target_z, target_yaw;
  std::string tf_err;
  if (!resolve_target_to_odom(
        request->frame_id, request->x, request->y, request->z, request->yaw,
        target_x, target_y, target_z, target_yaw, tf_err)) {
    RCLCPP_ERROR(this->get_logger(), "Navigate failed: %s", tf_err.c_str());
    response->success = false;
    response->message = tf_err;
    last_error_ = tf_err;
    return;
  }

  double speed = request->speed > 0.05 ? static_cast<double>(request->speed) : default_speed_;

  std::string safety_err;
  if (!safety_guard_->validate_setpoint(target_x, target_y, target_z, target_yaw, speed, safety_err)) {
    RCLCPP_ERROR(this->get_logger(), "Navigate rejected by safety guard: %s", safety_err.c_str());
    response->success = false;
    response->message = safety_err;
    last_error_ = safety_err;
    return;
  }

  if (request->auto_arm || !cmd_sender_->is_armed()) {
    std::string arm_msg;
    cmd_sender_->arm(arm_msg);
  }

  heartbeat_mgr_->set_position_control(true);
  heartbeat_mgr_->start();
  trajectory_tracker_->set_streaming_enabled(true);

  std::string mode_msg;
  cmd_sender_->set_offboard_mode(mode_msg);

  trajectory_tracker_->set_target(target_x, target_y, target_z, target_yaw, speed);
  flight_state_ = FlightState::NAVIGATING;

  RCLCPP_INFO(
    this->get_logger(),
    "Navigate target set: [%.2f, %.2f, %.2f], yaw: %.2f rad, speed: %.2f m/s (from frame '%s')",
    target_x, target_y, target_z, target_yaw, speed, request->frame_id.c_str());

  response->success = true;
  response->message = "Goal accepted (Target in odom: [" +
                      std::to_string(target_x) + ", " +
                      std::to_string(target_y) + ", " +
                      std::to_string(target_z) + "])";
}

void OffboardNode::handle_set_altitude(
  const std::shared_ptr<pickme_interfaces::srv::SetAltitude::Request> request,
  std::shared_ptr<pickme_interfaces::srv::SetAltitude::Response> response) {
  double target_z = request->z;

  if (request->frame_id == "body" || request->frame_id == "base_link") {
    target_z = current_pos_.z + request->z;
  } else if (!request->frame_id.empty() && request->frame_id != odom_frame_) {
    double dummy_x, dummy_y, dummy_yaw;
    std::string tf_err;
    if (!resolve_target_to_odom(
          request->frame_id, 0.0, 0.0, request->z, 0.0,
          dummy_x, dummy_y, target_z, dummy_yaw, tf_err)) {
      response->success = false;
      response->message = tf_err;
      return;
    }
  }

  if (target_z < (geofence_min_alt_ - 0.3) || target_z > geofence_max_alt_) {
    response->success = false;
    response->message = "Altitude " + std::to_string(target_z) + "m outside geofence limits";
    return;
  }

  trajectory_tracker_->set_target_altitude(target_z);
  flight_state_ = FlightState::NAVIGATING;
  response->success = true;
  response->message = "Altitude set to " + std::to_string(target_z) + "m";
}

void OffboardNode::handle_set_yaw(
  const std::shared_ptr<pickme_interfaces::srv::SetYaw::Request> request,
  std::shared_ptr<pickme_interfaces::srv::SetYaw::Response> response) {
  double target_yaw = request->yaw;

  if (request->frame_id == "body" || request->frame_id == "base_link") {
    target_yaw = current_pos_.yaw + request->yaw;
  } else if (!request->frame_id.empty() && request->frame_id != odom_frame_) {
    double dummy_x, dummy_y, dummy_z;
    std::string tf_err;
    if (!resolve_target_to_odom(
          request->frame_id, 0.0, 0.0, 0.0, request->yaw,
          dummy_x, dummy_y, dummy_z, target_yaw, tf_err)) {
      response->success = false;
      response->message = tf_err;
      return;
    }
  }

  while (target_yaw > PI) target_yaw -= 2.0 * PI;
  while (target_yaw < -PI) target_yaw += 2.0 * PI;

  trajectory_tracker_->set_target_yaw(target_yaw);
  flight_state_ = FlightState::NAVIGATING;
  response->success = true;
  response->message = "Target yaw set to " + std::to_string(target_yaw) + " rad";
}

void OffboardNode::handle_get_telemetry(
  const std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Request> request,
  std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Response> response) {
  response->connected = cmd_sender_->is_connected();
  response->armed = cmd_sender_->is_armed();
  response->mode = cmd_sender_->get_nav_state_str();

  if (request->frame_id.empty() || request->frame_id == odom_frame_) {
    response->x = static_cast<float>(current_pos_.x);
    response->y = static_cast<float>(current_pos_.y);
    response->z = static_cast<float>(current_pos_.z);
    response->yaw = static_cast<float>(current_pos_.yaw);
    response->vx = static_cast<float>(current_vel_.x);
    response->vy = static_cast<float>(current_vel_.y);
    response->vz = static_cast<float>(current_vel_.z);
  } else {
    try {
      geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
        request->frame_id, odom_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));

      geometry_msgs::msg::PoseStamped pose_in, pose_out;
      pose_in.header.frame_id = odom_frame_;
      pose_in.pose.position.x = current_pos_.x;
      pose_in.pose.position.y = current_pos_.y;
      pose_in.pose.position.z = current_pos_.z;

      tf2::Quaternion q_in;
      q_in.setRPY(0.0, 0.0, current_pos_.yaw);
      pose_in.pose.orientation = tf2::toMsg(q_in);

      tf2::doTransform(pose_in, pose_out, tf_stamped);

      response->x = static_cast<float>(pose_out.pose.position.x);
      response->y = static_cast<float>(pose_out.pose.position.y);
      response->z = static_cast<float>(pose_out.pose.position.z);

      tf2::Quaternion q_out;
      tf2::fromMsg(pose_out.pose.orientation, q_out);
      tf2::Matrix3x3 m(q_out);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      response->yaw = static_cast<float>(yaw);

      response->vx = static_cast<float>(current_vel_.x);
      response->vy = static_cast<float>(current_vel_.y);
      response->vz = static_cast<float>(current_vel_.z);
    } catch (const tf2::TransformException &) {
      response->x = static_cast<float>(current_pos_.x);
      response->y = static_cast<float>(current_pos_.y);
      response->z = static_cast<float>(current_pos_.z);
      response->yaw = static_cast<float>(current_pos_.yaw);
      response->vx = static_cast<float>(current_vel_.x);
      response->vy = static_cast<float>(current_vel_.y);
      response->vz = static_cast<float>(current_vel_.z);
    }
  }

  response->battery_voltage = cmd_sender_->get_battery_voltage();
}

std::string OffboardNode::flight_state_to_str(FlightState state) const {
  switch (state) {
    case FlightState::DISARMED: return "DISARMED";
    case FlightState::ARMING: return "ARMING";
    case FlightState::ARMED: return "ARMED";
    case FlightState::TAKEOFF: return "TAKEOFF";
    case FlightState::NAVIGATING: return "NAVIGATING";
    case FlightState::HOVERING: return "HOVERING";
    case FlightState::LANDING: return "LANDING";
    case FlightState::EMERGENCY: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

}  // namespace pickme_offboard

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pickme_offboard::OffboardNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
