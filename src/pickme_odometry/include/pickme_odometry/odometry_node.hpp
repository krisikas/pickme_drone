/**
 * OdometryNode — Centralized coordinate frame (TF2) and odometry broadcaster.
 *
 * Converts PX4 aerospace coordinates (NED/FRD) to ROS 2 standard (ENU/FLU per REP-103/REP-105).
 * Broadcasts dynamic odom->base_link TF, static sensor frames, and standard /odom topic.
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace pickme_odometry {

constexpr double SQRT2_2 = 0.70710678118654752440;
constexpr double PI = 3.14159265358979323846;

inline geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

class OdometryNode : public rclcpp::Node {
public:
  explicit OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_static_transforms();
  void handle_vehicle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  // === Frame & Parameter Members ===
  std::string odom_frame_;
  std::string base_frame_;
  std::string body_frame_;
  std::string camera_frame_;
  std::string camera_depth_frame_;
  std::string camera_depth_optical_frame_;
  std::string camera_color_frame_;
  std::string camera_color_optical_frame_;
  std::string camera_optical_frame_;
  std::string imu_frame_;

  double cam_x_, cam_y_, cam_z_;
  double cam_roll_, cam_pitch_, cam_yaw_;
  bool publish_camera_tf_;
  bool publish_odom_;
  std::string odom_topic_;

  // === ROS Entities ===
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
};

}  // namespace pickme_odometry
