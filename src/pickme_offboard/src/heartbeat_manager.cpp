/**
 * HeartbeatManager — Implementation of 10 Hz Offboard heartbeat stream.
 */

#include "pickme_offboard/heartbeat_manager.hpp"

namespace pickme_offboard {

HeartbeatManager::HeartbeatManager(rclcpp::Node * node)
: node_(node) {
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .transient_local();

  offboard_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", px4_qos);

  start();
}

void HeartbeatManager::start() {
  if (!active_) {
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HeartbeatManager::send_heartbeat, this));
    active_ = true;
  }
}

void HeartbeatManager::stop() {
  if (active_ && timer_) {
    timer_->cancel();
    active_ = false;
  }
}

void HeartbeatManager::set_position_control(bool enable) {
  position_control_enabled_ = enable;
}

bool HeartbeatManager::is_active() const {
  return active_;
}

void HeartbeatManager::send_heartbeat() {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
  msg.position = position_control_enabled_;
  msg.velocity = position_control_enabled_;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  offboard_mode_pub_->publish(msg);
}

}  // namespace pickme_offboard
