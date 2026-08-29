/**
 * SafetyGuard — Implementation of flight safety constraints and validation.
 */

#include "pickme_offboard/safety_guard.hpp"

namespace pickme_offboard {

SafetyGuard::SafetyGuard(
  double geofence_radius,
  double geofence_min_alt,
  double geofence_max_alt,
  double max_speed,
  double max_yaw_rate)
: geofence_radius_(geofence_radius),
  geofence_min_alt_(geofence_min_alt),
  geofence_max_alt_(geofence_max_alt),
  max_speed_(max_speed),
  max_yaw_rate_(max_yaw_rate) {}

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
                    " m/s exceeds limit (0.0, " + std::to_string(max_speed_) + "] m/s";
    return false;
  }

  double horiz_dist = std::sqrt(x * x + y * y);
  if (horiz_dist > geofence_radius_) {
    error_message = "Rejected: target position (" + std::to_string(horiz_dist) +
                    "m from origin) exceeds horizontal geofence radius (" +
                    std::to_string(geofence_radius_) + "m)";
    return false;
  }

  // Allow small margin (-0.3m) below min_alt to handle ground contact and sensor drift
  constexpr double ALT_TOLERANCE = 0.3;
  if (z < (geofence_min_alt_ - ALT_TOLERANCE) || z > geofence_max_alt_) {
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

void SafetyGuard::set_limits(double max_speed, double max_yaw_rate) {
  max_speed_ = max_speed;
  max_yaw_rate_ = max_yaw_rate;
}

double SafetyGuard::get_max_speed() const {
  return max_speed_;
}

double SafetyGuard::get_max_yaw_rate() const {
  return max_yaw_rate_;
}

}  // namespace pickme_offboard
