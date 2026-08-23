/**
 * CommandSender — Sends vehicle commands to PX4 and monitors command acknowledgments (ACK) and control modes.
 */

#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace pickme_offboard {

struct CommandAckResult {
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

  bool send_command_and_wait(
    uint32_t command,
    float param1, float param2, float param3,
    float param4, float param5, float param6, float param7,
    std::string & out_msg,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

  bool arm(std::string & out_msg, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  bool disarm(std::string & out_msg, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  bool set_offboard_mode(std::string & out_msg, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  bool land(std::string & out_msg, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

  bool is_armed() const;
  bool is_connected() const;
  bool is_offboard() const;
  uint8_t get_nav_state() const;
  std::string get_nav_state_str() const;

  std::string format_ack_error(uint8_t result) const;

private:
  void handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void handle_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void handle_control_mode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);

  rclcpp::Node * node_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  uint32_t waiting_command_{0};
  uint8_t last_ack_result_{255};
  bool ack_received_{false};

  px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;
  px4_msgs::msg::VehicleControlMode::SharedPtr control_mode_;
  std::unordered_map<uint32_t, CommandAckResult> last_acks_;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_mode_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace pickme_offboard
