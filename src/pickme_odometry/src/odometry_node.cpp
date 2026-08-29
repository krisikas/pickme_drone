/**
 * OdometryNode — Implementation of TF2 coordinate transformations and odometry publishing.
 */

#include "pickme_odometry/odometry_node.hpp"

namespace pickme_odometry {

OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: Node("odometry_node", options) {
  // === Parameter Declarations ===
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  body_frame_ = this->declare_parameter<std::string>("body_frame", "body");
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
  camera_depth_frame_ = this->declare_parameter<std::string>("camera_depth_frame", "camera_depth_frame");
  camera_depth_optical_frame_ = this->declare_parameter<std::string>("camera_depth_optical_frame", "camera_depth_optical_frame");
  camera_color_frame_ = this->declare_parameter<std::string>("camera_color_frame", "camera_color_frame");
  camera_color_optical_frame_ = this->declare_parameter<std::string>("camera_color_optical_frame", "camera_color_optical_frame");
  camera_optical_frame_ = this->declare_parameter<std::string>("camera_optical_frame", "camera_optical_frame");
  imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu_link");

  cam_x_ = this->declare_parameter<double>("camera_offset_x", 0.12);
  cam_y_ = this->declare_parameter<double>("camera_offset_y", 0.03);
  cam_z_ = this->declare_parameter<double>("camera_offset_z", 0.242);
  cam_roll_ = this->declare_parameter<double>("camera_roll", 0.0);
  cam_pitch_ = this->declare_parameter<double>("camera_pitch", 0.0);
  cam_yaw_ = this->declare_parameter<double>("camera_yaw", 0.0);

  publish_camera_tf_ = this->declare_parameter<bool>("publish_camera_tf", true);
  publish_odom_ = this->declare_parameter<bool>("publish_odom_topic", true);
  odom_topic_ = this->declare_parameter<std::string>("odom_topic_name", "/odom");

  // === Broadcasters & Publishers ===
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  if (publish_odom_) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  }

  // === PX4 Subscription ===
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .transient_local();

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", px4_qos,
    std::bind(&OdometryNode::handle_vehicle_odometry, this, std::placeholders::_1));

  // === Publish Initial Static Tree ===
  publish_static_transforms();

  RCLCPP_INFO(
    this->get_logger(),
    "OdometryNode initialized: '%s' -> '%s' -> '%s'",
    odom_frame_.c_str(), base_frame_.c_str(), camera_frame_.c_str());
}

void OdometryNode::publish_static_transforms() {
  auto now = this->get_clock()->now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // 1. base_link -> camera_link (Mount offset in FLU)
  geometry_msgs::msg::TransformStamped t_cam;
  t_cam.header.stamp = now;
  t_cam.header.frame_id = base_frame_;
  t_cam.child_frame_id = camera_frame_;
  t_cam.transform.translation.x = cam_x_;
  t_cam.transform.translation.y = cam_y_;
  t_cam.transform.translation.z = cam_z_;
  t_cam.transform.rotation = euler_to_quaternion(cam_roll_, cam_pitch_, cam_yaw_);
  transforms.push_back(t_cam);

  // 2. Camera Optical Frames (REP-103 standard RDF)
  if (publish_camera_tf_) {
    auto opt_q = euler_to_quaternion(-PI / 2.0, 0.0, -PI / 2.0);

    // 2a. camera_link -> camera_depth_frame (FLU)
    geometry_msgs::msg::TransformStamped t_depth_frame;
    t_depth_frame.header.stamp = now;
    t_depth_frame.header.frame_id = camera_frame_;
    t_depth_frame.child_frame_id = camera_depth_frame_;
    t_depth_frame.transform.rotation.w = 1.0;
    transforms.push_back(t_depth_frame);

    // 2b. camera_depth_frame -> camera_depth_optical_frame (RDF)
    geometry_msgs::msg::TransformStamped t_depth_opt;
    t_depth_opt.header.stamp = now;
    t_depth_opt.header.frame_id = camera_depth_frame_;
    t_depth_opt.child_frame_id = camera_depth_optical_frame_;
    t_depth_opt.transform.rotation = opt_q;
    transforms.push_back(t_depth_opt);

    // 2c. camera_link -> camera_color_frame (FLU, Y = -15mm RGB offset)
    geometry_msgs::msg::TransformStamped t_color_frame;
    t_color_frame.header.stamp = now;
    t_color_frame.header.frame_id = camera_frame_;
    t_color_frame.child_frame_id = camera_color_frame_;
    t_color_frame.transform.translation.y = -0.015;
    t_color_frame.transform.rotation.w = 1.0;
    transforms.push_back(t_color_frame);

    // 2d. camera_color_frame -> camera_color_optical_frame (RDF)
    geometry_msgs::msg::TransformStamped t_color_opt;
    t_color_opt.header.stamp = now;
    t_color_opt.header.frame_id = camera_color_frame_;
    t_color_opt.child_frame_id = camera_color_optical_frame_;
    t_color_opt.transform.rotation = opt_q;
    transforms.push_back(t_color_opt);

    // 2e. camera_link -> camera_optical_frame (RDF generic alias)
    geometry_msgs::msg::TransformStamped t_opt_alias;
    t_opt_alias.header.stamp = now;
    t_opt_alias.header.frame_id = camera_frame_;
    t_opt_alias.child_frame_id = camera_optical_frame_;
    t_opt_alias.transform.rotation = opt_q;
    transforms.push_back(t_opt_alias);
  }

  // 3. base_link -> imu_link (FLU)
  geometry_msgs::msg::TransformStamped t_imu;
  t_imu.header.stamp = now;
  t_imu.header.frame_id = base_frame_;
  t_imu.child_frame_id = imu_frame_;
  t_imu.transform.rotation.w = 1.0;
  transforms.push_back(t_imu);

  static_tf_broadcaster_->sendTransform(transforms);
}

void OdometryNode::handle_vehicle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  auto now = this->get_clock()->now();

  // 1. Position conversion (PX4 NED -> ROS ENU)
  const double pos_x = static_cast<double>(msg->position[1]);
  const double pos_y = static_cast<double>(msg->position[0]);
  const double pos_z = -static_cast<double>(msg->position[2]);

  // 2. Orientation conversion (PX4 NED/FRD -> ROS ENU/FLU)
  const double w_px4 = static_cast<double>(msg->q[0]);
  const double x_px4 = static_cast<double>(msg->q[1]);
  const double y_px4 = static_cast<double>(msg->q[2]);
  const double z_px4 = static_cast<double>(msg->q[3]);

  double w_ros = SQRT2_2 * (w_px4 + z_px4);
  double x_ros = SQRT2_2 * (x_px4 + y_px4);
  double y_ros = SQRT2_2 * (x_px4 - y_px4);
  double z_ros = SQRT2_2 * (w_px4 - z_px4);

  const double norm = std::sqrt(w_ros * w_ros + x_ros * x_ros + y_ros * y_ros + z_ros * z_ros);
  if (norm > 1e-6) {
    const double inv_norm = 1.0 / norm;
    w_ros *= inv_norm;
    x_ros *= inv_norm;
    y_ros *= inv_norm;
    z_ros *= inv_norm;
  } else {
    w_ros = 1.0;
    x_ros = y_ros = z_ros = 0.0;
  }

  // 3. Broadcast Dynamic TF (odom -> base_link and odom -> body)
  geometry_msgs::msg::TransformStamped t_base;
  t_base.header.stamp = now;
  t_base.header.frame_id = odom_frame_;
  t_base.child_frame_id = base_frame_;
  t_base.transform.translation.x = pos_x;
  t_base.transform.translation.y = pos_y;
  t_base.transform.translation.z = pos_z;
  t_base.transform.rotation.x = x_ros;
  t_base.transform.rotation.y = y_ros;
  t_base.transform.rotation.z = z_ros;
  t_base.transform.rotation.w = w_ros;

  // Extract horizontal yaw for body frame (roll=0, pitch=0)
  double yaw = std::atan2(2.0 * (w_ros * z_ros + x_ros * y_ros), 1.0 - 2.0 * (y_ros * y_ros + z_ros * z_ros));
  geometry_msgs::msg::TransformStamped t_body;
  t_body.header.stamp = now;
  t_body.header.frame_id = odom_frame_;
  t_body.child_frame_id = body_frame_;
  t_body.transform.translation.x = pos_x;
  t_body.transform.translation.y = pos_y;
  t_body.transform.translation.z = pos_z;
  t_body.transform.rotation = euler_to_quaternion(0.0, 0.0, yaw);

  tf_broadcaster_->sendTransform(t_base);
  tf_broadcaster_->sendTransform(t_body);

  // 4. Publish Standard ROS 2 Odometry Message
  if (odom_pub_) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_msg.pose.pose.position.z = pos_z;
    odom_msg.pose.pose.orientation.x = x_ros;
    odom_msg.pose.pose.orientation.y = y_ros;
    odom_msg.pose.pose.orientation.z = z_ros;
    odom_msg.pose.pose.orientation.w = w_ros;

    // Linear velocity (NED -> ENU)
    odom_msg.twist.twist.linear.x = static_cast<double>(msg->velocity[1]);
    odom_msg.twist.twist.linear.y = static_cast<double>(msg->velocity[0]);
    odom_msg.twist.twist.linear.z = -static_cast<double>(msg->velocity[2]);

    // Angular velocity (FRD -> FLU)
    odom_msg.twist.twist.angular.x = static_cast<double>(msg->angular_velocity[0]);
    odom_msg.twist.twist.angular.y = -static_cast<double>(msg->angular_velocity[1]);
    odom_msg.twist.twist.angular.z = -static_cast<double>(msg->angular_velocity[2]);

    odom_pub_->publish(odom_msg);
  }
}

}  // namespace pickme_odometry

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pickme_odometry::OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
