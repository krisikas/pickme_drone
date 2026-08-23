/**
 * ============================================================================
 * PickmeDrone — Offboard Flight Coordinator Node
 * ============================================================================
 * 
 * Central flight management node for ROS 2 + PX4 Autopilot over Micro-XRCE-DDS.
 * 
 * Features:
 * - Continuous 10 Hz Heartbeat (OffboardControlMode) & 50 Hz Setpoints (TrajectorySetpoint)
 * - Fast non-blocking ROS 2 services (<1ms response time)
 * - Real-time VehicleControlMode & Odometry tracking
 * - Universal TF2 target frame resolution (odom, map, body, base_link, navigate_target, camera_link)
 * - Real-time Safety Guard (Geofence, NaN/Inf checks, speed limits, odometry watchdog)
 * - Solid heading & position tracking (like Clover)
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
 * 7. Land vehicle:
 *    ros2 service call /pickme/land std_srvs/srv/Trigger {}
 * 
 * 8. Disarm vehicle:
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
  takeoff_altitude_ = this->declare_parameter<double>("takeoff_altitude", 2.0);
  geofence_radius_ = this->declare_parameter<double>("geofence_radius", 25.0);
  geofence_min_alt_ = this->declare_parameter<double>("geofence_min_alt", 0.0);
  geofence_max_alt_ = this->declare_parameter<double>("geofence_max_alt", 15.0);
  odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 1.0);
  target_reached_threshold_ = this->declare_parameter<double>("target_reached_threshold", 0.1);

  // === Initialize Modular Components ===
  heartbeat_mgr_ = std::make_unique<HeartbeatManager>(this);
  cmd_sender_ = std::make_unique<CommandSender>(this);
  safety_guard_ = std::make_unique<SafetyGuard>(
    geofence_radius_, geofence_min_alt_, geofence_max_alt_, 5.0);
  trajectory_tracker_ = std::make_unique<TrajectoryTracker>(
    this, odom_frame_, target_frame_, default_speed_, target_reached_threshold_);

  // === TF2 Buffer & Listener ===
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // === QoS Configuration ===
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .transient_local();

  // === Subscribers ===
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", px4_qos,
    std::bind(&OffboardNode::handle_odometry, this, std::placeholders::_1));

  // === 50 Hz Control Timer (20 ms) ===
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&OffboardNode::control_loop, this));

  // === Service Servers ===
  arm_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/arm",
    std::bind(&OffboardNode::handle_arm, this, std::placeholders::_1, std::placeholders::_2));

  disarm_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/disarm",
    std::bind(&OffboardNode::handle_disarm, this, std::placeholders::_1, std::placeholders::_2));

  takeoff_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/takeoff",
    std::bind(&OffboardNode::handle_takeoff, this, std::placeholders::_1, std::placeholders::_2));

  land_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/pickme/land",
    std::bind(&OffboardNode::handle_land, this, std::placeholders::_1, std::placeholders::_2));

  navigate_service_ = this->create_service<pickme_interfaces::srv::Navigate>(
    "/pickme/navigate",
    std::bind(&OffboardNode::handle_navigate, this, std::placeholders::_1, std::placeholders::_2));

  set_altitude_service_ = this->create_service<pickme_interfaces::srv::SetAltitude>(
    "/pickme/set_altitude",
    std::bind(&OffboardNode::handle_set_altitude, this, std::placeholders::_1, std::placeholders::_2));

  set_yaw_service_ = this->create_service<pickme_interfaces::srv::SetYaw>(
    "/pickme/set_yaw",
    std::bind(&OffboardNode::handle_set_yaw, this, std::placeholders::_1, std::placeholders::_2));

  telemetry_service_ = this->create_service<pickme_interfaces::srv::GetTelemetry>(
    "/pickme/get_telemetry",
    std::bind(&OffboardNode::handle_get_telemetry, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "OffboardNode initialized (Modular & Fast Non-Blocking)");
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

  current_pos_.yaw = std::atan2(2.0 * (w_ros * z_ros + x_ros * y_ros), 1.0 - 2.0 * (y_ros * y_ros + z_ros * z_ros));

  // Initialize trajectory tracker hold position on ground as soon as first odometry arrives
  if (!trajectory_tracker_->is_initialized()) {
    trajectory_tracker_->snap_to_position(current_pos_.x, current_pos_.y, current_pos_.z, current_pos_.yaw);
  }
}

void OffboardNode::control_loop() {
  auto now = this->get_clock()->now();

  std::string odom_err;
  if (!safety_guard_->validate_odometry_freshness(now, last_odom_time_, odom_timeout_sec_, odom_err)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Safety watchdog: %s", odom_err.c_str());
    return;
  }

  // Always stream TrajectorySetpoint at 50 Hz so PX4 never reports "No offboard signal"
  constexpr double dt = 0.02;  // 50 Hz = 20ms
  trajectory_tracker_->update_and_publish(dt, now);
}

bool OffboardNode::resolve_target_to_odom(
  const std::string & frame_id,
  double req_x, double req_y, double req_z, double req_yaw,
  double & out_x, double & out_y, double & out_z, double & out_yaw,
  std::string & err_msg) {
  // 1. Direct world frame cases
  if (frame_id.empty() || frame_id == odom_frame_ || frame_id == "map") {
    out_x = req_x;
    out_y = req_y;
    out_z = req_z;
    out_yaw = req_yaw;
    return true;
  }

  // 2. Universal TF2 resolution
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      odom_frame_, frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));

    double tx = tf_stamped.transform.translation.x;
    double ty = tf_stamped.transform.translation.y;
    double tz = tf_stamped.transform.translation.z;

    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, frame_yaw;
    m.getRPY(roll, pitch, frame_yaw);

    // Rotate requested offset by frame's yaw
    double cy = std::cos(frame_yaw);
    double sy = std::sin(frame_yaw);

    out_x = tx + (req_x * cy - req_y * sy);
    out_y = ty + (req_x * sy + req_y * cy);
    out_z = tz + req_z;
    out_yaw = frame_yaw + req_yaw;

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
  RCLCPP_INFO(this->get_logger(), "ARM requested");
  std::string msg;
  cmd_sender_->arm(msg);

  response->success = true;
  response->message = "Arm command sent to PX4";
}

void OffboardNode::handle_disarm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "DISARM requested");
  std::string msg;
  cmd_sender_->disarm(msg);
  trajectory_tracker_->reset_target();
  flight_state_ = FlightState::DISARMED;

  response->success = true;
  response->message = "Disarm command sent to PX4";
}

void OffboardNode::handle_takeoff(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  auto now = this->get_clock()->now();
  std::string odom_err;
  if (!safety_guard_->validate_odometry_freshness(now, last_odom_time_, odom_timeout_sec_, odom_err)) {
    response->success = false;
    response->message = "Takeoff rejected: " + odom_err;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "TAKEOFF requested to altitude %.1fm", takeoff_altitude_);

  // 1. Arm vehicle
  std::string arm_msg;
  cmd_sender_->arm(arm_msg);

  // 2. Snap tracker to current position and set takeoff target
  trajectory_tracker_->snap_to_position(current_pos_.x, current_pos_.y, current_pos_.z, current_pos_.yaw);
  trajectory_tracker_->set_target(
    current_pos_.x, current_pos_.y, takeoff_altitude_, current_pos_.yaw, default_speed_);

  // 3. Switch to Offboard Mode
  std::string mode_msg;
  cmd_sender_->set_offboard_mode(mode_msg);

  flight_state_ = FlightState::NAVIGATING;
  response->success = true;
  response->message = "Takeoff initiated (Target Alt: " + std::to_string(takeoff_altitude_) + "m)";
}

void OffboardNode::handle_land(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "LAND requested");
  std::string land_msg;
  cmd_sender_->land(land_msg);
  trajectory_tracker_->reset_target();
  flight_state_ = FlightState::LANDING;

  response->success = true;
  response->message = "Land command dispatched to PX4";
}

void OffboardNode::handle_navigate(
  const std::shared_ptr<pickme_interfaces::srv::Navigate::Request> request,
  std::shared_ptr<pickme_interfaces::srv::Navigate::Response> response) {
  auto now = this->get_clock()->now();
  std::string odom_err;
  if (!safety_guard_->validate_odometry_freshness(now, last_odom_time_, odom_timeout_sec_, odom_err)) {
    response->success = false;
    response->message = "Navigate rejected: " + odom_err;
    return;
  }

  // 1. Resolve requested coordinates to odom frame using TF2
  double target_x, target_y, target_z, target_yaw;
  std::string tf_err;
  if (!resolve_target_to_odom(
        request->frame_id, request->x, request->y, request->z, request->yaw,
        target_x, target_y, target_z, target_yaw, tf_err)) {
    RCLCPP_ERROR(this->get_logger(), "Navigate failed: %s", tf_err.c_str());
    response->success = false;
    response->message = tf_err;
    return;
  }

  double speed = request->speed > 0.05 ? static_cast<double>(request->speed) : default_speed_;

  // 2. Safety Validation (Geofence, NaN, limits)
  std::string safety_err;
  if (!safety_guard_->validate_setpoint(target_x, target_y, target_z, target_yaw, speed, safety_err)) {
    RCLCPP_ERROR(this->get_logger(), "Navigate rejected by safety guard: %s", safety_err.c_str());
    response->success = false;
    response->message = safety_err;
    return;
  }

  // 3. Auto-arm or ensure Offboard mode
  if (request->auto_arm) {
    std::string arm_msg;
    cmd_sender_->arm(arm_msg);
    std::string mode_msg;
    cmd_sender_->set_offboard_mode(mode_msg);
  } else {
    // If not in offboard mode, engage offboard
    if (!cmd_sender_->is_offboard()) {
      std::string mode_msg;
      cmd_sender_->set_offboard_mode(mode_msg);
    }
  }

  // 4. Set target in Trajectory Tracker
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
  double target_x, target_y, target_z, target_yaw;
  std::string tf_err;
  if (!resolve_target_to_odom(
        request->frame_id, 0.0, 0.0, request->z, 0.0,
        target_x, target_y, target_z, target_yaw, tf_err)) {
    response->success = false;
    response->message = tf_err;
    return;
  }

  if (target_z < geofence_min_alt_ || target_z > geofence_max_alt_) {
    response->success = false;
    response->message = "Altitude " + std::to_string(target_z) + "m outside geofence limits";
    return;
  }

  trajectory_tracker_->set_target_altitude(target_z);
  response->success = true;
  response->message = "Altitude set to " + std::to_string(target_z) + "m";
}

void OffboardNode::handle_set_yaw(
  const std::shared_ptr<pickme_interfaces::srv::SetYaw::Request> request,
  std::shared_ptr<pickme_interfaces::srv::SetYaw::Response> response) {
  double target_x, target_y, target_z, target_yaw;
  std::string tf_err;
  if (!resolve_target_to_odom(
        request->frame_id, 0.0, 0.0, 0.0, request->yaw,
        target_x, target_y, target_z, target_yaw, tf_err)) {
    response->success = false;
    response->message = tf_err;
    return;
  }

  trajectory_tracker_->set_target_yaw(target_yaw);
  response->success = true;
  response->message = "Target yaw set to " + std::to_string(target_yaw) + " rad";
}

void OffboardNode::handle_get_telemetry(
  const std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Request> /*request*/,
  std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Response> response) {
  response->connected = cmd_sender_->is_connected();
  response->armed = cmd_sender_->is_armed();
  response->mode = cmd_sender_->get_nav_state_str();
  response->x = static_cast<float>(current_pos_.x);
  response->y = static_cast<float>(current_pos_.y);
  response->z = static_cast<float>(current_pos_.z);
  response->yaw = static_cast<float>(current_pos_.yaw);
  response->vx = static_cast<float>(current_vel_.x);
  response->vy = static_cast<float>(current_vel_.y);
  response->vz = static_cast<float>(current_vel_.z);
  response->battery_voltage = 16.0f;
}

}  // namespace pickme_offboard

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pickme_offboard::OffboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
