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
  enable_vo_bridge_ = this->declare_parameter<bool>("enable_visual_odometry_bridge", true);
  vo_topic_ = this->declare_parameter<std::string>("visual_odometry_topic", "/visual_odometry");

  // === Broadcasters & Publishers ===
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  if (publish_odom_) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  }

  // === PX4 Subscription & Publishers ===
  auto px4_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .transient_local();
  auto px4_sub_qos = rclcpp::SensorDataQoS();

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", px4_sub_qos,
    std::bind(&OdometryNode::handle_vehicle_odometry, this, std::placeholders::_1));

  if (enable_vo_bridge_) {
    vo_px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", px4_pub_qos);

    vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      vo_topic_, px4_sub_qos,
      std::bind(&OdometryNode::handle_visual_odometry, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Visual Odometry Bridge active: '%s' -> '/fmu/in/vehicle_visual_odometry'",
      vo_topic_.c_str());
  }

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

void OdometryNode::handle_visual_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!vo_px4_pub_) {
    return;
  }

  // 1. Position conversion (ROS ENU -> PX4 NED)
  const float pos_x_ned = static_cast<float>(msg->pose.pose.position.y);
  const float pos_y_ned = static_cast<float>(msg->pose.pose.position.x);
  const float pos_z_ned = -static_cast<float>(msg->pose.pose.position.z);

  // 2. Orientation conversion (ROS ENU/FLU -> PX4 NED/FRD)
  const double w_r = msg->pose.pose.orientation.w;
  const double x_r = msg->pose.pose.orientation.x;
  const double y_r = msg->pose.pose.orientation.y;
  const double z_r = msg->pose.pose.orientation.z;

  float w_px4 = static_cast<float>(SQRT2_2 * (w_r + z_r));
  float x_px4 = static_cast<float>(SQRT2_2 * (x_r + y_r));
  float y_px4 = static_cast<float>(SQRT2_2 * (x_r - y_r));
  float z_px4 = static_cast<float>(SQRT2_2 * (w_r - z_r));

  const float norm = std::sqrt(w_px4 * w_px4 + x_px4 * x_px4 + y_px4 * y_px4 + z_px4 * z_px4);
  if (norm > 1e-6f) {
    const float inv_norm = 1.0f / norm;
    w_px4 *= inv_norm;
    x_px4 *= inv_norm;
    y_px4 *= inv_norm;
    z_px4 *= inv_norm;
  } else {
    w_px4 = 1.0f;
    x_px4 = y_px4 = z_px4 = 0.0f;
  }

  // 3. Velocity conversion (FLU -> FRD)
  const float v_x_frd = static_cast<float>(msg->twist.twist.linear.x);
  const float v_y_frd = -static_cast<float>(msg->twist.twist.linear.y);
  const float v_z_frd = -static_cast<float>(msg->twist.twist.linear.z);

  const float w_x_frd = static_cast<float>(msg->twist.twist.angular.x);
  const float w_y_frd = -static_cast<float>(msg->twist.twist.angular.y);
  const float w_z_frd = -static_cast<float>(msg->twist.twist.angular.z);

  // 4. Construct PX4 VehicleOdometry Message
  px4_msgs::msg::VehicleOdometry px4_vo;
  px4_vo.timestamp = this->get_clock()->now().nanoseconds() / 1000ULL;
  px4_vo.timestamp_sample = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
  px4_vo.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

  px4_vo.position[0] = pos_x_ned;
  px4_vo.position[1] = pos_y_ned;
  px4_vo.position[2] = pos_z_ned;

  px4_vo.q[0] = w_px4;
  px4_vo.q[1] = x_px4;
  px4_vo.q[2] = y_px4;
  px4_vo.q[3] = z_px4;

  px4_vo.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  px4_vo.velocity[0] = v_x_frd;
  px4_vo.velocity[1] = v_y_frd;
  px4_vo.velocity[2] = v_z_frd;

  px4_vo.angular_velocity[0] = w_x_frd;
  px4_vo.angular_velocity[1] = w_y_frd;
  px4_vo.angular_velocity[2] = w_z_frd;

  // Variances (fallbacks if covariance matrix is unfilled)
  px4_vo.position_variance[0] = static_cast<float>(msg->pose.covariance[0] > 0.0 ? msg->pose.covariance[0] : 0.01f);
  px4_vo.position_variance[1] = static_cast<float>(msg->pose.covariance[7] > 0.0 ? msg->pose.covariance[7] : 0.01f);
  px4_vo.position_variance[2] = static_cast<float>(msg->pose.covariance[14] > 0.0 ? msg->pose.covariance[14] : 0.01f);

  px4_vo.orientation_variance[0] = static_cast<float>(msg->pose.covariance[21] > 0.0 ? msg->pose.covariance[21] : 0.02f);
  px4_vo.orientation_variance[1] = static_cast<float>(msg->pose.covariance[28] > 0.0 ? msg->pose.covariance[28] : 0.02f);
  px4_vo.orientation_variance[2] = static_cast<float>(msg->pose.covariance[35] > 0.0 ? msg->pose.covariance[35] : 0.02f);

  px4_vo.velocity_variance[0] = static_cast<float>(msg->twist.covariance[0] > 0.0 ? msg->twist.covariance[0] : 0.01f);
  px4_vo.velocity_variance[1] = static_cast<float>(msg->twist.covariance[7] > 0.0 ? msg->twist.covariance[7] : 0.01f);
  px4_vo.velocity_variance[2] = static_cast<float>(msg->twist.covariance[14] > 0.0 ? msg->twist.covariance[14] : 0.01f);

  vo_px4_pub_->publish(px4_vo);
}

}  // namespace pickme_odometry

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pickme_odometry::OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
