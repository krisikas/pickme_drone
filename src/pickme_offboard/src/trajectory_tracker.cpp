/**
 * TrajectoryTracker — Implementation of 50 Hz closed-loop trajectory interpolation and TF broadcasting.
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
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .transient_local();

  trajectory_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", px4_qos);
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
  remaining_distance_ = 0.0;
}

void TrajectoryTracker::set_streaming_enabled(bool enabled) {
  streaming_enabled_ = enabled;
}

void TrajectoryTracker::update_and_publish(
  double dt,
  const rclcpp::Time & now,
  const Waypoint3D & drone_pos,
  const Waypoint3D & drone_vel) {
  if (!initialized_ || !streaming_enabled_) {
    return;
  }

  // 1. Calculate actual distance from real drone position to target setpoint
  double drone_dx = target_setpoint_.x - drone_pos.x;
  double drone_dy = target_setpoint_.y - drone_pos.y;
  double drone_dz = target_setpoint_.z - drone_pos.z;
  remaining_distance_ = std::sqrt(drone_dx * drone_dx + drone_dy * drone_dy + drone_dz * drone_dz);

  // Calculate current drone linear speed
  double drone_speed = std::sqrt(
    drone_vel.x * drone_vel.x +
    drone_vel.y * drone_vel.y +
    drone_vel.z * drone_vel.z);

  // 2. Calculate vector from current interpolated setpoint to target setpoint
  double setpoint_dx = target_setpoint_.x - current_setpoint_.x;
  double setpoint_dy = target_setpoint_.y - current_setpoint_.y;
  double setpoint_dz = target_setpoint_.z - current_setpoint_.z;
  double setpoint_dist = std::sqrt(
    setpoint_dx * setpoint_dx +
    setpoint_dy * setpoint_dy +
    setpoint_dz * setpoint_dz);

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;

  if (has_active_target_ && setpoint_dist > 0.001) {
    double step = current_target_speed_ * dt;
    if (step >= setpoint_dist) {
      current_setpoint_.x = target_setpoint_.x;
      current_setpoint_.y = target_setpoint_.y;
      current_setpoint_.z = target_setpoint_.z;
    } else {
      double dir_x = setpoint_dx / setpoint_dist;
      double dir_y = setpoint_dy / setpoint_dist;
      double dir_z = setpoint_dz / setpoint_dist;

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
  }

  // 3. Lag Limiter: prevent setpoint from drifting too far ahead of the physical drone
  double lead_dx = current_setpoint_.x - drone_pos.x;
  double lead_dy = current_setpoint_.y - drone_pos.y;
  double lead_dz = current_setpoint_.z - drone_pos.z;
  double lead_dist = std::sqrt(lead_dx * lead_dx + lead_dy * lead_dy + lead_dz * lead_dz);
  const double max_lead_dist = std::max(0.6, current_target_speed_ * 0.8);

  if (lead_dist > max_lead_dist && lead_dist > 0.001) {
    double scale = max_lead_dist / lead_dist;
    current_setpoint_.x = drone_pos.x + lead_dx * scale;
    current_setpoint_.y = drone_pos.y + lead_dy * scale;
    current_setpoint_.z = drone_pos.z + lead_dz * scale;
  }

  // 4. Determine if target is reached based on real drone position and velocity
  if (has_active_target_) {
    if (remaining_distance_ <= target_reached_threshold_ && drone_speed < 0.25) {
      target_reached_ = true;
    } else {
      target_reached_ = false;
    }
  } else {
    target_reached_ = true;
  }

  // 5. Yaw tracking directly handled by PX4 attitude loop
  current_setpoint_.yaw = target_setpoint_.yaw;

  // 6. Convert ENU setpoint -> PX4 NED for TrajectorySetpoint
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

  // Acceleration & Jerk set to NaN
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

  // 7. Broadcast dynamic TF frame for "navigate_target"
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

bool TrajectoryTracker::is_streaming_enabled() const {
  return streaming_enabled_;
}

double TrajectoryTracker::get_remaining_distance() const {
  return remaining_distance_;
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
