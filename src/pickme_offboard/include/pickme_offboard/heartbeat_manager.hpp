/**
 * HeartbeatManager — Manages 10 Hz Offboard heartbeat stream to PX4.
 */

#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

namespace pickme_offboard {

class HeartbeatManager {
public:
  explicit HeartbeatManager(rclcpp::Node * node);

  void start();
  void stop();
  void send_heartbeat();
  void set_position_control(bool enable);
  bool is_active() const;

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool active_{false};
  bool position_control_enabled_{true};
};

}  // namespace pickme_offboard
