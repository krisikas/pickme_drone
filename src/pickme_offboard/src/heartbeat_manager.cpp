/**
 * HeartbeatManager — Implementation of 10 Hz Offboard heartbeat stream.
 */

#include "pickme_offboard/heartbeat_manager.hpp"

namespace pickme_offboard {

HeartbeatManager::HeartbeatManager(rclcpp::Node * node)
: node_(node) {
  offboard_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);

  // Default: start heartbeat immediately so PX4 knows Offboard mode is available
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

void HeartbeatManager::send_heartbeat() {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  offboard_mode_pub_->publish(msg);
}

}  // namespace pickme_offboard
