/**
 * CommandSender — Implementation of PX4 vehicle command transmission and telemetry monitoring.
 */

#include "pickme_offboard/command_sender.hpp"

namespace pickme_offboard {

CommandSender::CommandSender(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr cb_group)
: node_(node) {
  auto px4_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .transient_local();
  auto px4_sub_qos = rclcpp::SensorDataQoS();

  rclcpp::SubscriptionOptions sub_options;
  if (cb_group) {
    sub_options.callback_group = cb_group;
  }

  cmd_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", px4_pub_qos);

  ack_sub_ = node_->create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", px4_sub_qos,
    std::bind(&CommandSender::handle_command_ack, this, std::placeholders::_1),
    sub_options);

  status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", px4_sub_qos,
    std::bind(&CommandSender::handle_vehicle_status, this, std::placeholders::_1),
    sub_options);

  control_mode_sub_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", px4_sub_qos,
    std::bind(&CommandSender::handle_control_mode, this, std::placeholders::_1),
    sub_options);

  battery_sub_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status", px4_sub_qos,
    std::bind(&CommandSender::handle_battery_status, this, std::placeholders::_1),
    sub_options);
}

void CommandSender::send_command(
  uint32_t command,
  float param1, float param2, float param3,
  float param4, float param5, float param6, float param7) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
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

  cmd_pub_->publish(msg);
}

bool CommandSender::arm(std::string & out_msg) {
  if (!is_connected()) {
    RCLCPP_WARN(node_->get_logger(), "Flight controller status not yet confirmed; sending ARM anyway");
  }
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  out_msg = "Arm command dispatched to PX4";
  return true;
}

bool CommandSender::disarm(std::string & out_msg) {
  if (!is_connected()) {
    RCLCPP_WARN(node_->get_logger(), "Flight controller status not yet confirmed; sending DISARM anyway");
  }
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
  out_msg = "Disarm command dispatched to PX4";
  return true;
}

bool CommandSender::set_offboard_mode(std::string & out_msg) {
  if (!is_connected()) {
    RCLCPP_WARN(node_->get_logger(), "Flight controller status not yet confirmed; sending OFFBOARD mode anyway");
  }
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  out_msg = "Offboard mode command dispatched to PX4";
  return true;
}

bool CommandSender::land(std::string & out_msg) {
  if (!is_connected()) {
    RCLCPP_WARN(node_->get_logger(), "Flight controller status not yet confirmed; sending LAND anyway");
  }
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  out_msg = "Land command dispatched to PX4";
  return true;
}

bool CommandSender::is_connected_unlocked() const {
  auto now = node_->get_clock()->now();
  if (last_control_mode_time_.nanoseconds() > 0 && (now - last_control_mode_time_).seconds() < 3.0) {
    return true;
  }
  if (last_status_time_.nanoseconds() > 0 && (now - last_status_time_).seconds() < 3.0) {
    return true;
  }
  return false;
}

bool CommandSender::is_armed() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (control_mode_) {
    return control_mode_->flag_armed;
  }
  if (vehicle_status_) {
    return vehicle_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }
  return false;
}

bool CommandSender::is_connected() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_connected_unlocked();
}

bool CommandSender::is_offboard() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (control_mode_) {
    return control_mode_->flag_control_offboard_enabled;
  }
  if (vehicle_status_) {
    return vehicle_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  return false;
}

uint8_t CommandSender::get_nav_state() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return vehicle_status_ ? vehicle_status_->nav_state : 255;
}

std::string CommandSender::get_nav_state_str() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (control_mode_ && control_mode_->flag_control_offboard_enabled) {
    return "OFFBOARD";
  }
  if (!vehicle_status_) {
    return is_connected_unlocked() ? "CONNECTED" : "DISCONNECTED";
  }
  switch (vehicle_status_->nav_state) {
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL: return "MANUAL";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL: return "ALTCTL";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL: return "POSCTL";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "AUTO_MISSION";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return "HOLD";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return "RTL";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return "LAND";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "TAKEOFF";
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
    default: return "OTHER(" + std::to_string(vehicle_status_->nav_state) + ")";
  }
}

float CommandSender::get_battery_voltage() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return battery_voltage_;
}

float CommandSender::get_battery_remaining() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return battery_remaining_;
}

bool CommandSender::get_last_ack(uint32_t command, uint8_t & out_result, rclcpp::Time & out_time) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = last_acks_.find(command);
  if (it != last_acks_.end()) {
    out_result = it->second.result;
    out_time = it->second.timestamp;
    return true;
  }
  return false;
}

void CommandSender::handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CommandAckRecord ack;
  ack.result = msg->result;
  ack.timestamp = node_->get_clock()->now();
  last_acks_[msg->command] = ack;

  if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED &&
      msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PX4 Command %u response: %u (%s)",
      msg->command, msg->result, format_ack_error(msg->result).c_str());
  }
}

void CommandSender::handle_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  vehicle_status_ = msg;
  last_status_time_ = node_->get_clock()->now();
}

void CommandSender::handle_control_mode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  control_mode_ = msg;
  last_control_mode_time_ = node_->get_clock()->now();
}

void CommandSender::handle_battery_status(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  battery_voltage_ = msg->voltage_v;
  battery_remaining_ = msg->remaining;
}

std::string CommandSender::format_ack_error(uint8_t result) const {
  switch (result) {
    case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
      return "Accepted";
    case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
      return "Temporarily rejected (Preflight checks or calibrating)";
    case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
      return "Denied (Safety check failed or system health failure)";
    case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
      return "Unsupported command";
    case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
      return "Internal execution failure";
    case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:
      return "In progress";
    default:
      return "Result code (" + std::to_string(result) + ")";
  }
}

}  // namespace pickme_offboard
