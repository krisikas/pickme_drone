/**
 * OffboardNode — Centralized flight control and trajectory setpoint streamer.
 */

#include "pickme_offboard/offboard_node.hpp"

namespace pickme_offboard {

constexpr double SQRT2_2 = 0.70710678118654752440;

OffboardNode::OffboardNode(const rclcpp::NodeOptions & options)
: Node("offboard_node", options) {
  // === Parameter Declarations ===
  default_speed_ = this->declare_parameter<double>("default_speed", 1.0);
  takeoff_altitude_ = this->declare_parameter<double>("takeoff_altitude", 2.0);
  geofence_radius_ = this->declare_parameter<double>("geofence_radius", 25.0);
  geofence_min_alt_ = this->declare_parameter<double>("geofence_min_alt", 0.0);
  geofence_max_alt_ = this->declare_parameter<double>("geofence_max_alt", 15.0);
  odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 1.0);
  target_reached_threshold_ = this->declare_parameter<double>("target_reached_threshold", 0.1);

  current_target_speed_ = default_speed_;

  // === QoS Configuration ===
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .transient_local();

  // === Publishers ===
  offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);
  trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  // === Subscribers ===
  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", px4_qos,
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
      vehicle_status_ = msg;
    });

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", px4_qos,
    [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      last_odom_time_ = this->get_clock()->now();

      // Convert PX4 NED -> ROS ENU
      current_pos_.x = static_cast<double>(msg->position[1]);
      current_pos_.y = static_cast<double>(msg->position[0]);
      current_pos_.z = -static_cast<double>(msg->position[2]);

      current_vel_.x = static_cast<double>(msg->velocity[1]);
      current_vel_.y = static_cast<double>(msg->velocity[0]);
      current_vel_.z = -static_cast<double>(msg->velocity[2]);

      // Orientation (NED/FRD -> ENU/FLU yaw)
      const double w_px4 = static_cast<double>(msg->q[0]);
      const double x_px4 = static_cast<double>(msg->q[1]);
      const double y_px4 = static_cast<double>(msg->q[2]);
      const double z_px4 = static_cast<double>(msg->q[3]);

      double w_ros = SQRT2_2 * (w_px4 + z_px4);
      double x_ros = SQRT2_2 * (x_px4 + y_px4);
      double y_ros = SQRT2_2 * (x_px4 - y_px4);
      double z_ros = SQRT2_2 * (w_px4 - z_px4);

      // Extract yaw from ROS quaternion
      current_pos_.yaw = std::atan2(2.0 * (w_ros * z_ros + x_ros * y_ros), 1.0 - 2.0 * (y_ros * y_ros + z_ros * z_ros));
    });

  // === Services ===
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

  // === Timers ===
  // 10 Hz Heartbeat timer
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&OffboardNode::publish_heartbeat, this));

  // 50 Hz Control & Trajectory loop (20 ms)
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&OffboardNode::update_setpoint_tracking, this));

  RCLCPP_INFO(this->get_logger(), "OffboardNode initialized and ready for flight missions");
}

// === Heartbeat Publisher ===
void OffboardNode::publish_heartbeat() {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  offboard_mode_pub_->publish(msg);
}

// === Trajectory Tracker Loop ===
void OffboardNode::update_setpoint_tracking() {
  if (!offboard_engaged_) {
    return;
  }

  // Watchdog: check odometry freshness
  auto now = this->get_clock()->now();
  if (last_odom_time_.nanoseconds() > 0 && (now - last_odom_time_).seconds() > odom_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odometry timeout! Holding current setpoint.");
    return;
  }

  constexpr double dt = 0.02;  // 50 Hz = 20ms

  // Calculate distance vector to target setpoint (in ENU)
  double dx = target_setpoint_.x - current_setpoint_.x;
  double dy = target_setpoint_.y - current_setpoint_.y;
  double dz = target_setpoint_.z - current_setpoint_.z;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;

  if (dist > target_reached_threshold_) {
    double step = current_target_speed_ * dt;
    if (step >= dist) {
      current_setpoint_.x = target_setpoint_.x;
      current_setpoint_.y = target_setpoint_.y;
      current_setpoint_.z = target_setpoint_.z;
    } else {
      double dir_x = dx / dist;
      double dir_y = dy / dist;
      double dir_z = dz / dist;

      current_setpoint_.x += dir_x * step;
      current_setpoint_.y += dir_y * step;
      current_setpoint_.z += dir_z * step;

      vx = dir_x * current_target_speed_;
      vy = dir_y * current_target_speed_;
      vz = dir_z * current_target_speed_;
    }
  } else {
    current_setpoint_.x = target_setpoint_.x;
    current_setpoint_.y = target_setpoint_.y;
    current_setpoint_.z = target_setpoint_.z;
    if (flight_state_ == FlightState::NAVIGATING) {
      flight_state_ = FlightState::HOVERING;
    }
  }

  // Track yaw smoothly
  double dyaw = target_setpoint_.yaw - current_setpoint_.yaw;
  while (dyaw > PI) dyaw -= 2.0 * PI;
  while (dyaw < -PI) dyaw += 2.0 * PI;

  constexpr double max_yaw_rate = 1.0;  // rad/s
  double yaw_step = max_yaw_rate * dt;
  if (std::abs(dyaw) <= yaw_step) {
    current_setpoint_.yaw = target_setpoint_.yaw;
  } else {
    current_setpoint_.yaw += (dyaw > 0 ? 1.0 : -1.0) * yaw_step;
  }

  // Convert ENU setpoint -> PX4 NED
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = now.nanoseconds() / 1000;

  // Position: ENU -> NED
  msg.position[0] = static_cast<float>(current_setpoint_.y);   // NED X = North = ENU Y
  msg.position[1] = static_cast<float>(current_setpoint_.x);   // NED Y = East  = ENU X
  msg.position[2] = -static_cast<float>(current_setpoint_.z);  // NED Z = Down  = -ENU Z

  // Velocity: ENU -> NED
  msg.velocity[0] = static_cast<float>(vy);
  msg.velocity[1] = static_cast<float>(vx);
  msg.velocity[2] = -static_cast<float>(vz);

  // Yaw: ENU (0=East, CCW) -> NED (0=North, CW)
  // yaw_ned = -yaw_enu + PI/2
  double yaw_ned = -current_setpoint_.yaw + (PI / 2.0);
  while (yaw_ned > PI) yaw_ned -= 2.0 * PI;
  while (yaw_ned < -PI) yaw_ned += 2.0 * PI;
  msg.yaw = static_cast<float>(yaw_ned);

  trajectory_pub_->publish(msg);
}

// === Command Helper ===
void OffboardNode::publish_vehicle_command(
  uint32_t command, float param1, float param2, float param3,
  float param4, float param5, float param6, float param7) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  vehicle_command_pub_->publish(msg);
}

// === Safety Validation ===
bool OffboardNode::validate_position_safety(
  double x, double y, double z, double yaw, double speed, std::string & error_msg) {
  if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(yaw) || std::isnan(speed)) {
    error_msg = "Coordinates contain NaN";
    return false;
  }
  if (std::isinf(x) || std::isinf(y) || std::isinf(z) || std::isinf(yaw) || std::isinf(speed)) {
    error_msg = "Coordinates contain Infinity";
    return false;
  }

  double horiz_dist = std::sqrt(x * x + y * y);
  if (horiz_dist > geofence_radius_) {
    error_msg = "Target exceeds horizontal geofence radius (" + std::to_string(geofence_radius_) + "m)";
    return false;
  }
  if (z < geofence_min_alt_ || z > geofence_max_alt_) {
    error_msg = "Target altitude outside geofence limits [" + std::to_string(geofence_min_alt_) + ", " +
                std::to_string(geofence_max_alt_) + "m]";
    return false;
  }
  return true;
}

// === Service Handlers ===
void OffboardNode::handle_arm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "ARM requested");
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  response->success = true;
  response->message = "Arm command sent to PX4";
}

void OffboardNode::handle_disarm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "DISARM requested");
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
  offboard_engaged_ = false;
  flight_state_ = FlightState::DISARMED;
  response->success = true;
  response->message = "Disarm command sent to PX4";
}

void OffboardNode::handle_takeoff(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "TAKEOFF requested to altitude %.1fm", takeoff_altitude_);

  // 1. Arm
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);

  // 2. Set initial target right above current position
  current_setpoint_ = current_pos_;
  target_setpoint_ = current_pos_;
  target_setpoint_.z = takeoff_altitude_;
  current_target_speed_ = default_speed_;

  // 3. Switch to Offboard Mode (VEHICLE_CMD_DO_SET_MODE, 1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 = PX4_CUSTOM_MAIN_MODE_OFFBOARD)
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);

  offboard_engaged_ = true;
  flight_state_ = FlightState::TAKING_OFF;

  response->success = true;
  response->message = "Takeoff sequence initiated (Target Alt: " + std::to_string(takeoff_altitude_) + "m)";
}

void OffboardNode::handle_land(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "LAND requested");
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  offboard_engaged_ = false;
  flight_state_ = FlightState::LANDING;
  response->success = true;
  response->message = "Land command sent to PX4";
}

void OffboardNode::handle_navigate(
  const std::shared_ptr<pickme_interfaces::srv::Navigate::Request> request,
  std::shared_ptr<pickme_interfaces::srv::Navigate::Response> response) {
  double target_x = request->x;
  double target_y = request->y;
  double target_z = request->z;
  double target_yaw = request->yaw;
  double speed = request->speed > 0.05 ? static_cast<double>(request->speed) : default_speed_;

  // Transform coordinates if requested in body frame
  if (request->frame_id == "body" || request->frame_id == "base_link") {
    double cy = std::cos(current_pos_.yaw);
    double sy = std::sin(current_pos_.yaw);
    target_x = current_pos_.x + (request->x * cy - request->y * sy);
    target_y = current_pos_.y + (request->x * sy + request->y * cy);
    target_z = current_pos_.z + request->z;
    target_yaw = current_pos_.yaw + request->yaw;
  }

  // Safety verification
  std::string err;
  if (!validate_position_safety(target_x, target_y, target_z, target_yaw, speed, err)) {
    RCLCPP_ERROR(this->get_logger(), "Navigate rejected: %s", err.c_str());
    response->success = false;
    response->message = err;
    return;
  }

  // Auto-arm and engage offboard if required
  if (request->auto_arm) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  if (!offboard_engaged_) {
    current_setpoint_ = current_pos_;
    offboard_engaged_ = true;
  }

  target_setpoint_.x = target_x;
  target_setpoint_.y = target_y;
  target_setpoint_.z = target_z;
  target_setpoint_.yaw = target_yaw;
  current_target_speed_ = speed;
  flight_state_ = FlightState::NAVIGATING;

  RCLCPP_INFO(
    this->get_logger(),
    "Navigate goal set: [%.2f, %.2f, %.2f], yaw: %.2f rad, speed: %.2f m/s",
    target_x, target_y, target_z, target_yaw, speed);

  response->success = true;
  response->message = "Goal accepted";
}

void OffboardNode::handle_set_altitude(
  const std::shared_ptr<pickme_interfaces::srv::SetAltitude::Request> request,
  std::shared_ptr<pickme_interfaces::srv::SetAltitude::Response> response) {
  double target_z = request->z;
  if (request->frame_id == "body" || request->frame_id == "base_link") {
    target_z = current_pos_.z + request->z;
  }

  if (target_z < geofence_min_alt_ || target_z > geofence_max_alt_) {
    response->success = false;
    response->message = "Altitude outside geofence limits";
    return;
  }

  target_setpoint_.z = target_z;
  response->success = true;
  response->message = "Altitude set to " + std::to_string(target_z) + "m";
}

void OffboardNode::handle_set_yaw(
  const std::shared_ptr<pickme_interfaces::srv::SetYaw::Request> request,
  std::shared_ptr<pickme_interfaces::srv::SetYaw::Response> response) {
  double target_yaw = request->yaw;
  if (request->frame_id == "body" || request->frame_id == "base_link") {
    target_yaw = current_pos_.yaw + request->yaw;
  }

  while (target_yaw > PI) target_yaw -= 2.0 * PI;
  while (target_yaw < -PI) target_yaw += 2.0 * PI;

  target_setpoint_.yaw = target_yaw;
  response->success = true;
  response->message = "Target yaw set";
}

void OffboardNode::handle_get_telemetry(
  const std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Request> /*request*/,
  std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Response> response) {
  response->connected = (last_odom_time_.nanoseconds() > 0);
  response->armed = (vehicle_status_ && vehicle_status_->arming_state == 2);
  response->mode = offboard_engaged_ ? "OFFBOARD" : "MANUAL/HOLD";
  response->x = static_cast<float>(current_pos_.x);
  response->y = static_cast<float>(current_pos_.y);
  response->z = static_cast<float>(current_pos_.z);
  response->yaw = static_cast<float>(current_pos_.yaw);
  response->vx = static_cast<float>(current_vel_.x);
  response->vy = static_cast<float>(current_vel_.y);
  response->vz = static_cast<float>(current_vel_.z);
  response->battery_voltage = 16.0f;  // Default simulation voltage
}

}  // namespace pickme_offboard

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pickme_offboard::OffboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
