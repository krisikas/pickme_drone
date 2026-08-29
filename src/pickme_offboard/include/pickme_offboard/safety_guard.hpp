/**
 * SafetyGuard — Flight safety verification, Geofence checks, and odometry watchdog.
 */

#pragma once

#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace pickme_offboard {

class SafetyGuard {
public:
  SafetyGuard(
    double geofence_radius,
    double geofence_min_alt,
    double geofence_max_alt,
    double max_speed,
    double max_yaw_rate = 1.5);

  bool validate_setpoint(
    double x, double y, double z, double yaw, double speed,
    std::string & error_message) const;

  bool validate_odometry_freshness(
    const rclcpp::Time & now,
    const rclcpp::Time & last_odom_time,
    double timeout_sec,
    std::string & error_message) const;

  void update_geofence(double radius, double min_alt, double max_alt);
  void set_limits(double max_speed, double max_yaw_rate);

  double get_max_speed() const;
  double get_max_yaw_rate() const;

private:
  double geofence_radius_;
  double geofence_min_alt_;
  double geofence_max_alt_;
  double max_speed_;
  double max_yaw_rate_;
};

}  // namespace pickme_offboard
