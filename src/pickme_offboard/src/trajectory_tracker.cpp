/**
 * TrajectoryTracker — Implementation of 50 Hz trajectory interpolation and TF broadcasting.
 */

#include "pickme_offboard/trajectory_tracker.hpp"

namespace pickme_offboard {

TrajectoryTracker::TrajectoryTracker(
  rclcpp::Node * node,
  const std::string & odom_frame,
  const std::string & target_frame,
  double default_speed,
  double target_reached_threshold)
: node_(node),
  odom_frame_(odom_frame),
  target_frame_(target_frame),
  default_speed_(default_speed),
  target_reached_threshold_(target_reached_threshold),
  current_target_speed_(default_speed) {
  trajectory_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

void TrajectoryTracker::set_target(double x, double y, double z, double yaw, double speed) {
  target_setpoint_.x = x;
  target_setpoint_.y = y;
  target_setpoint_.z = z;
  target_setpoint_.yaw = yaw;
  current_target_speed_ = speed > 0.05 ? speed : default_speed_;
  target_reached_ = false;
  has_active_target_ = true;

  if (!initialized_) {
    current_setpoint_ = target_setpoint_;
    initialized_ = true;
  }
}

void TrajectoryTracker::snap_to_position(double x, double y, double z, double yaw) {
  current_setpoint_.x = x;
  current_setpoint_.y = y;
  current_setpoint_.z = z;
  current_setpoint_.yaw = yaw;

  target_setpoint_ = current_setpoint_;
  target_reached_ = true;
  initialized_ = true;
}

void TrajectoryTracker::set_target_altitude(double z) {
  target_setpoint_.z = z;
  target_reached_ = false;
  has_active_target_ = true;
}

void TrajectoryTracker::set_target_yaw(double yaw) {
  target_setpoint_.yaw = yaw;
  target_reached_ = false;
  has_active_target_ = true;
}

void TrajectoryTracker::reset_target() {
  has_active_target_ = false;
  target_reached_ = true;
}

void TrajectoryTracker::update_and_publish(double dt, const rclcpp::Time & now) {
  if (!initialized_) {
    return;
  }

  // 1. Calculate distance vector to target setpoint (in ENU)
  double dx = target_setpoint_.x - current_setpoint_.x;
  double dy = target_setpoint_.y - current_setpoint_.y;
  double dz = target_setpoint_.z - current_setpoint_.z;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;

  if (has_active_target_ && dist > target_reached_threshold_) {
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
    target_reached_ = false;
  } else {
    current_setpoint_.x = target_setpoint_.x;
    current_setpoint_.y = target_setpoint_.y;
    current_setpoint_.z = target_setpoint_.z;
    target_reached_ = true;
  }

  // 2. Fast, solid yaw tracking directly handled by PX4 attitude loop (like Clover)
  current_setpoint_.yaw = target_setpoint_.yaw;

  // 3. Convert ENU setpoint -> PX4 NED for TrajectorySetpoint
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = now.nanoseconds() / 1000;

  // Position: ENU -> NED
  msg.position[0] = static_cast<float>(current_setpoint_.y);   // NED X = North = ENU Y
  msg.position[1] = static_cast<float>(current_setpoint_.x);   // NED Y = East  = ENU X
  msg.position[2] = -static_cast<float>(current_setpoint_.z);  // NED Z = Down  = -ENU Z

  // Velocity: ENU -> NED (Feed-Forward)
  msg.velocity[0] = static_cast<float>(vy);
  msg.velocity[1] = static_cast<float>(vx);
  msg.velocity[2] = -static_cast<float>(vz);

  // Acceleration & Jerk set to NaN (let PX4 internal PID compute feed-forward)
  msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
  msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
  msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();
  msg.jerk[0] = std::numeric_limits<float>::quiet_NaN();
  msg.jerk[1] = std::numeric_limits<float>::quiet_NaN();
  msg.jerk[2] = std::numeric_limits<float>::quiet_NaN();

  // Yaw: ENU (0=East, CCW) -> NED (0=North, CW)
  double yaw_ned = -current_setpoint_.yaw + (PI / 2.0);
  while (yaw_ned > PI) yaw_ned -= 2.0 * PI;
  while (yaw_ned < -PI) yaw_ned += 2.0 * PI;
  msg.yaw = static_cast<float>(yaw_ned);
  msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

  trajectory_pub_->publish(msg);

  // 4. Broadcast dynamic TF frame for "navigate_target" ONLY if active target is valid
  if (has_active_target_) {
    geometry_msgs::msg::TransformStamped t_target;
    t_target.header.stamp = now;
    t_target.header.frame_id = odom_frame_;
    t_target.child_frame_id = target_frame_;
    t_target.transform.translation.x = target_setpoint_.x;
    t_target.transform.translation.y = target_setpoint_.y;
    t_target.transform.translation.z = target_setpoint_.z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, target_setpoint_.yaw);
    t_target.transform.rotation.x = q.x();
    t_target.transform.rotation.y = q.y();
    t_target.transform.rotation.z = q.z();
    t_target.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t_target);
  }
}

bool TrajectoryTracker::is_target_reached() const {
  return target_reached_;
}

bool TrajectoryTracker::is_initialized() const {
  return initialized_;
}

bool TrajectoryTracker::has_active_target() const {
  return has_active_target_;
}

Waypoint3D TrajectoryTracker::get_current_setpoint() const {
  return current_setpoint_;
}

Waypoint3D TrajectoryTracker::get_target_setpoint() const {
  return target_setpoint_;
}

double TrajectoryTracker::get_target_speed() const {
  return current_target_speed_;
}

}  // namespace pickme_offboard
