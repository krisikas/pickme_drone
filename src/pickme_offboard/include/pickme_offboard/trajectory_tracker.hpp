/**
 * TrajectoryTracker — 50 Hz trajectory interpolation, feed-forward velocity, and navigate_target TF broadcaster.
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace pickme_offboard {

constexpr double PI = 3.14159265358979323846;

struct Waypoint3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

class TrajectoryTracker {
public:
  TrajectoryTracker(
    rclcpp::Node * node,
    const std::string & odom_frame,
    const std::string & target_frame,
    double default_speed,
    double target_reached_threshold);

  void set_target(double x, double y, double z, double yaw, double speed);
  void snap_to_position(double x, double y, double z, double yaw);
  void set_target_altitude(double z);
  void set_target_yaw(double yaw);
  void reset_target();

  void update_and_publish(double dt, const rclcpp::Time & now);

  bool is_target_reached() const;
  bool is_initialized() const;
  bool has_active_target() const;
  Waypoint3D get_current_setpoint() const;
  Waypoint3D get_target_setpoint() const;
  double get_target_speed() const;

private:
  rclcpp::Node * node_;
  std::string odom_frame_;
  std::string target_frame_;
  double default_speed_;
  double target_reached_threshold_;
  double max_yaw_rate_{1.0};  // rad/s

  Waypoint3D current_setpoint_{};
  Waypoint3D target_setpoint_{};
  double current_target_speed_{1.0};
  bool target_reached_{true};
  bool initialized_{false};
  bool has_active_target_{false};

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace pickme_offboard
