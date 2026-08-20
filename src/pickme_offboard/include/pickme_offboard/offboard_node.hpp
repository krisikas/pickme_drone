/**
 * OffboardNode — Centralized flight control coordinator and trajectory tracking engine.
 *
 * Implements PX4 Offboard communication protocol over Micro-XRCE-DDS:
 * - 10 Hz Heartbeat: OffboardControlMode
 * - 20-50 Hz Setpoint Tracker: TrajectorySetpoint with speed interpolation
 * - Safety Guard: NaN/Inf checks, Geofence, Odometry watchdog
 * - High-level flight services: /arm, /disarm, /takeoff, /land, /navigate, /set_altitude, /set_yaw, /get_telemetry
 */

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <pickme_interfaces/srv/navigate.hpp>
#include <pickme_interfaces/srv/set_altitude.hpp>
#include <pickme_interfaces/srv/set_yaw.hpp>
#include <pickme_interfaces/srv/get_telemetry.hpp>

namespace pickme_offboard {

constexpr double PI = 3.14159265358979323846;

enum class FlightState {
  DISARMED,
  ARMED,
  TAKING_OFF,
  HOVERING,
  NAVIGATING,
  LANDING,
  LANDED
};

struct Position3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

class OffboardNode : public rclcpp::Node {
public:
  explicit OffboardNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // === Flight Control Timers ===
  void publish_heartbeat();
  void update_setpoint_tracking();

  // === PX4 Command Helpers ===
  void publish_vehicle_command(
    uint32_t command,
    float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f,
    float param7 = 0.0f);

  // === Service Callbacks ===
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

  // === Safety & Validation ===
  bool validate_position_safety(double x, double y, double z, double yaw, double speed, std::string & error_msg);

  // === Parameters ===
  double default_speed_;
  double takeoff_altitude_;
  double geofence_radius_;
  double geofence_min_alt_;
  double geofence_max_alt_;
  double odom_timeout_sec_;
  double target_reached_threshold_;

  // === State Variables ===
  FlightState flight_state_{FlightState::DISARMED};
  Position3D current_pos_{};
  Position3D current_vel_{};
  Position3D current_setpoint_{};
  Position3D target_setpoint_{};
  double current_target_speed_{1.0};
  bool offboard_engaged_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  // === Subscriptions ===
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;

  // === Publishers ===
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  // === Services ===
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
  rclcpp::Service<pickme_interfaces::srv::Navigate>::SharedPtr navigate_service_;
  rclcpp::Service<pickme_interfaces::srv::SetAltitude>::SharedPtr set_altitude_service_;
  rclcpp::Service<pickme_interfaces::srv::SetYaw>::SharedPtr set_yaw_service_;
  rclcpp::Service<pickme_interfaces::srv::GetTelemetry>::SharedPtr telemetry_service_;

  // === Timers ===
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace pickme_offboard
