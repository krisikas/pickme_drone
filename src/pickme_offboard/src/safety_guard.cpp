/**
 * SafetyGuard — Implementation of flight safety constraints.
 */

#include "pickme_offboard/safety_guard.hpp"

namespace pickme_offboard {

SafetyGuard::SafetyGuard(
  double geofence_radius,
  double geofence_min_alt,
  double geofence_max_alt,
  double max_speed)
: geofence_radius_(geofence_radius),
  geofence_min_alt_(geofence_min_alt),
  geofence_max_alt_(geofence_max_alt),
  max_speed_(max_speed) {}

bool SafetyGuard::validate_setpoint(
  double x, double y, double z, double yaw, double speed,
  std::string & error_message) const {
  if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(yaw) || std::isnan(speed)) {
    error_message = "Rejected: coordinates contain NaN value";
    return false;
  }
  if (std::isinf(x) || std::isinf(y) || std::isinf(z) || std::isinf(yaw) || std::isinf(speed)) {
    error_message = "Rejected: coordinates contain Infinite value";
    return false;
  }

  if (speed <= 0.0 || speed > max_speed_) {
    error_message = "Rejected: requested speed " + std::to_string(speed) +
                    " m/s exceeds max limit [" + std::to_string(max_speed_) + " m/s]";
    return false;
  }

  double horiz_dist = std::sqrt(x * x + y * y);
  if (horiz_dist > geofence_radius_) {
    error_message = "Rejected: target position (" + std::to_string(horiz_dist) +
                    "m from origin) exceeds horizontal geofence radius (" +
                    std::to_string(geofence_radius_) + "m)";
    return false;
  }

  if (z < geofence_min_alt_ || z > geofence_max_alt_) {
    error_message = "Rejected: target altitude " + std::to_string(z) +
                    "m outside geofence corridor [" + std::to_string(geofence_min_alt_) +
                    ", " + std::to_string(geofence_max_alt_) + "m]";
    return false;
  }

  return true;
}

bool SafetyGuard::validate_odometry_freshness(
  const rclcpp::Time & now,
  const rclcpp::Time & last_odom_time,
  double timeout_sec,
  std::string & error_message) const {
  if (last_odom_time.nanoseconds() == 0) {
    error_message = "No odometry data received yet from PX4";
    return false;
  }
  double age = (now - last_odom_time).seconds();
  if (age > timeout_sec) {
    error_message = "Odometry stream timeout (" + std::to_string(age) + "s > " + std::to_string(timeout_sec) + "s)";
    return false;
  }
  return true;
}

void SafetyGuard::update_geofence(double radius, double min_alt, double max_alt) {
  geofence_radius_ = radius;
  geofence_min_alt_ = min_alt;
  geofence_max_alt_ = max_alt;
}

}  // namespace pickme_offboard
