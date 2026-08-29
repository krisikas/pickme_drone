/**
 * CommandSender — Sends vehicle commands to PX4 and monitors acknowledgments, status, and telemetry.
 */

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace pickme_offboard {

struct CommandAckRecord {
  uint8_t result{255};
  rclcpp::Time timestamp{0, 0, RCL_ROS_TIME};
};

class CommandSender {
public:
  explicit CommandSender(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr cb_group = nullptr);

  void send_command(
    uint32_t command,
    float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f,
    float param7 = 0.0f);

  bool arm(std::string & out_msg);
  bool disarm(std::string & out_msg);
  bool set_offboard_mode(std::string & out_msg);
  bool land(std::string & out_msg);

  bool is_armed() const;
  bool is_connected() const;
  bool is_offboard() const;
  uint8_t get_nav_state() const;
  std::string get_nav_state_str() const;

  float get_battery_voltage() const;
  float get_battery_remaining() const;

  bool get_last_ack(uint32_t command, uint8_t & out_result, rclcpp::Time & out_time) const;
  std::string format_ack_error(uint8_t result) const;

private:
  void handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void handle_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void handle_control_mode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void handle_battery_status(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

  rclcpp::Node * node_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;

  mutable std::mutex mutex_;
  px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;
  px4_msgs::msg::VehicleControlMode::SharedPtr control_mode_;
  std::unordered_map<uint32_t, CommandAckRecord> last_acks_;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_mode_time_{0, 0, RCL_ROS_TIME};
  float battery_voltage_{0.0f};
  float battery_remaining_{0.0f};
};

}  // namespace pickme_offboard
