/**
 * OffboardNode — Centralized flight coordinator and ROS 2 service provider.
 *
 * Coordinates:
 * - HeartbeatManager (10 Hz heartbeat)
 * - CommandSender (VehicleCommand dispatch & VehicleCommandAck feedback)
 * - SafetyGuard (Geofence, NaN check, odometry watchdog)
 * - TrajectoryTracker (50 Hz setpoint tracking & navigate_target TF)
 * - TF2 Frame Resolution (Universal frame_id support: odom, map, body, base_link, navigate_target, camera_link, etc.)
 */

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pickme_offboard/heartbeat_manager.hpp"
#include "pickme_offboard/command_sender.hpp"
#include "pickme_offboard/safety_guard.hpp"
#include "pickme_offboard/trajectory_tracker.hpp"

#include <pickme_interfaces/srv/navigate.hpp>
#include <pickme_interfaces/srv/set_altitude.hpp>
#include <pickme_interfaces/srv/set_yaw.hpp>
#include <pickme_interfaces/srv/get_telemetry.hpp>

namespace pickme_offboard {

enum class FlightState {
  DISARMED,
  ARMING,
  OFFBOARD_ENGAGED,
  NAVIGATING,
  HOVERING,
  LANDING
};

class OffboardNode : public rclcpp::Node {
public:
  explicit OffboardNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // === Odometry Subscriber ===
  void handle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  // === Control Loop ===
  void control_loop();

  // === Universal TF2 Frame Transformer ===
  bool resolve_target_to_odom(
    const std::string & frame_id,
    double req_x, double req_y, double req_z, double req_yaw,
    double & out_x, double & out_y, double & out_z, double & out_yaw,
    std::string & err_msg);

  // === Service Handlers ===
  void handle_arm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_disarm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_takeoff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_land(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_navigate(
    const std::shared_ptr<pickme_interfaces::srv::Navigate::Request> request,
    std::shared_ptr<pickme_interfaces::srv::Navigate::Response> response);

  void handle_set_altitude(
    const std::shared_ptr<pickme_interfaces::srv::SetAltitude::Request> request,
    std::shared_ptr<pickme_interfaces::srv::SetAltitude::Response> response);

  void handle_set_yaw(
    const std::shared_ptr<pickme_interfaces::srv::SetYaw::Request> request,
    std::shared_ptr<pickme_interfaces::srv::SetYaw::Response> response);

  void handle_get_telemetry(
    const std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Request> request,
    std::shared_ptr<pickme_interfaces::srv::GetTelemetry::Response> response);

  // === Parameters ===
  std::string odom_frame_;
  std::string base_frame_;
  std::string target_frame_;
  double default_speed_;
  double takeoff_altitude_;
  double geofence_radius_;
  double geofence_min_alt_;
  double geofence_max_alt_;
  double odom_timeout_sec_;
  double target_reached_threshold_;

  // === Modular Components ===
  std::unique_ptr<HeartbeatManager> heartbeat_mgr_;
  std::unique_ptr<CommandSender> cmd_sender_;
  std::unique_ptr<SafetyGuard> safety_guard_;
  std::unique_ptr<TrajectoryTracker> trajectory_tracker_;

  // === TF2 Listener ===
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // === State Variables ===
  FlightState flight_state_{FlightState::DISARMED};
  Waypoint3D current_pos_{};
  Waypoint3D current_vel_{};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  // === ROS Entities ===
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
  rclcpp::Service<pickme_interfaces::srv::Navigate>::SharedPtr navigate_service_;
  rclcpp::Service<pickme_interfaces::srv::SetAltitude>::SharedPtr set_altitude_service_;
  rclcpp::Service<pickme_interfaces::srv::SetYaw>::SharedPtr set_yaw_service_;
  rclcpp::Service<pickme_interfaces::srv::GetTelemetry>::SharedPtr telemetry_service_;
};

}  // namespace pickme_offboard
