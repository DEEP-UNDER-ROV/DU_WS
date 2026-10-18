#include "rov_mavros/slam_to_mavros.hpp"

VisionToMavros::VisionToMavros()
    : Node("vision_to_mavros_node"),
      latest_velocity_stamp(0, 0, RCL_ROS_TIME),
      velocity_received(false),
      last_publish_time(0, 0, RCL_ROS_TIME),
      first_pose_received(false),
      reset_counter(1),
      linear_accel_cov(0.01),
      angular_vel_cov(0.01),
      pose_jump_threshold(0.1) {
  this->navigationParameters();
  this->precisionLandParameters();
}

void VisionToMavros::navigationParameters(void) {
  camera_pose_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose",
                                                              10);
  body_path_publisher =
      this->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);
  vision_pose_cov_publisher =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/mavros/vision_pose/pose_cov", 10);
  vision_speed_publisher =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/mavros/vision_speed/speed_twist_cov", 10);

  this->declare_parameter<std::string>("pose_topic", "/orb_slam3/pose");
  this->get_parameter("pose_topic", pose_topic);
  this->declare_parameter<std::string>("velocity_topic", "/orb_slam3/velocity");
  this->get_parameter("velocity_topic", velocity_topic);
  this->declare_parameter<bool>("velocity_in_camera_frame", false);
  this->get_parameter("velocity_in_camera_frame", velocity_in_camera_frame);
  this->declare_parameter<double>("velocity_timeout", 0.5);
  this->get_parameter("velocity_timeout", velocity_timeout);
  this->declare_parameter<double>("output_rate", 30.0);
  this->get_parameter("output_rate", output_rate);
  this->declare_parameter<double>("gamma_world", -1.5707963);
  this->get_parameter("gamma_world", gamma_world);
  this->declare_parameter<double>("roll_cam", 0.0);
  this->get_parameter("roll_cam", roll_cam);
  this->declare_parameter<double>("pitch_cam", 0.349066);
  this->get_parameter("pitch_cam", pitch_cam);
  this->declare_parameter<double>("yaw_cam", 0.0);
  this->get_parameter("yaw_cam", yaw_cam);
  this->declare_parameter<double>("linear_accel_cov", 0.01);
  this->get_parameter("linear_accel_cov", linear_accel_cov);
  this->declare_parameter<double>("angular_vel_cov", 0.01);
  this->get_parameter("angular_vel_cov", angular_vel_cov);
  this->declare_parameter<double>("pose_jump_threshold", 0.1);
  this->get_parameter("pose_jump_threshold", pose_jump_threshold);

  auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, qos,
      std::bind(&VisionToMavros::poseCallback, this, std::placeholders::_1));

  velocity_subscriber =
      this->create_subscription<geometry_msgs::msg::TwistStamped>(
          velocity_topic, qos,
          std::bind(&VisionToMavros::velocityCallback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "pose_topic: %s", pose_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "velocity_topic: %s (%s frame)",
              velocity_topic.c_str(),
              velocity_in_camera_frame ? "camera" : "odom");
  RCLCPP_INFO(this->get_logger(), "output_rate: %.1f Hz (0 = every message)",
              output_rate);
  RCLCPP_INFO(this->get_logger(), "gamma_world: %.4f rad (%.1f deg)",
              gamma_world, gamma_world * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "roll_cam: %.4f rad (%.1f deg)", roll_cam,
              roll_cam * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "pitch_cam: %.4f rad (%.1f deg)", pitch_cam,
              pitch_cam * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "yaw_cam: %.4f rad (%.1f deg)", yaw_cam,
              yaw_cam * 180.0 / M_PI);
}

void VisionToMavros::precisionLandParameters(void) {
  this->declare_parameter<bool>("enable_precland", false);
  this->get_parameter("enable_precland", enable_precland);

  RCLCPP_INFO(this->get_logger(), "Precision landing: %s",
              enable_precland ? "enabled" : "disabled");

  if (enable_precland) {
    this->declare_parameter<std::string>("precland_target_frame_id",
                                         "/landing_target");
    this->get_parameter("precland_target_frame_id", precland_target_frame_id);
    this->declare_parameter<std::string>("precland_camera_frame_id",
                                         "/camera_fisheye2_optical_frame");
    this->get_parameter("precland_camera_frame_id", precland_camera_frame_id);

    RCLCPP_INFO(this->get_logger(), "precland_target_frame_id: %s",
                precland_target_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "precland_camera_frame_id: %s",
                precland_camera_frame_id.c_str());

    precland_msg_publisher =
        this->create_publisher<mavros_msgs::msg::LandingTarget>("landing_raw",
                                                                10);
  }
}

std::array<double, 36> VisionToMavros::calculatePoseCovariance(
    int confidence_level) {
  confidence_level = std::max(0, std::min(3, confidence_level));
  double cov_pose = linear_accel_cov * std::pow(10.0, 3 - confidence_level);
  double cov_orientation =
      angular_vel_cov * std::pow(10.0, 1 - confidence_level);

  std::array<double, 36> covariance = {};
  covariance[0] = cov_pose;
  covariance[7] = cov_pose;
  covariance[14] = cov_pose;
  covariance[21] = cov_orientation;
  covariance[28] = cov_orientation;
  covariance[35] = cov_orientation;
  return covariance;
}

std::array<double, 36> VisionToMavros::calculateTwistCovariance(
    int confidence_level) {
  confidence_level = std::max(0, std::min(3, confidence_level));
  double cov_vel = linear_accel_cov * std::pow(10.0, 3 - confidence_level);

  std::array<double, 36> covariance = {};
  covariance[0] = cov_vel;
  covariance[7] = cov_vel;
  covariance[14] = cov_vel;
  covariance[21] = 1e6;
  covariance[28] = 1e6;
  covariance[35] = 1e6;
  return covariance;
}

void VisionToMavros::detectPoseJump(const tf2::Vector3 &current_pos) {
  if (!first_pose_received) {
    prev_position_body = current_pos;
    first_pose_received = true;
    return;
  }

  tf2::Vector3 delta = current_pos - prev_position_body;
  double displacement = delta.length();

  if (displacement > pose_jump_threshold) {
    RCLCPP_WARN(this->get_logger(), "Pose jump detected! Displacement: %.3f m",
                displacement);
    reset_counter++;
    if (reset_counter == 0) reset_counter = 1;
    RCLCPP_INFO(this->get_logger(), "Reset counter: %d", reset_counter);
  }

  prev_position_body = current_pos;
}

void VisionToMavros::velocityCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  // Only the linear part is used; angular velocity is not needed by MAVROS
  latest_velocity.setValue(msg->twist.linear.x, msg->twist.linear.y,
                           msg->twist.linear.z);
  latest_velocity_stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
  velocity_received = true;
}

void VisionToMavros::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);

  // Rate limit on message stamps. A stamp going backwards (e.g. SLAM reset or
  // sim restart) is let through.
  if (output_rate > 0.0 && last_publish_time.nanoseconds() != 0) {
    double dt = (stamp - last_publish_time).seconds();
    if (dt >= 0.0 && dt < 1.0 / output_rate) return;
  }

  // Pose from the SLAM node, in its map frame
  tf2::fromMsg(msg->pose.position, position_orig);
  tf2::fromMsg(msg->pose.orientation, quat_cam);

  position_body.setX(cos(gamma_world) * position_orig.getX() +
                     sin(gamma_world) * position_orig.getY());
  position_body.setY(-sin(gamma_world) * position_orig.getX() +
                     cos(gamma_world) * position_orig.getY());
  position_body.setZ(position_orig.getZ());

  quat_cam_to_body.setRPY(roll_cam, pitch_cam, yaw_cam);
  quat_rot_z.setRPY(0, 0, -gamma_world);
  quat_body = quat_rot_z * quat_cam * quat_cam_to_body;
  quat_body.normalize();

  if (std::isnan(position_body.getX()) || std::isnan(position_body.getY()) ||
      std::isnan(position_body.getZ()) || std::isnan(quat_body.getX()) ||
      std::isnan(quat_body.getY()) || std::isnan(quat_body.getZ()) ||
      std::isnan(quat_body.getW())) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "NaN in pose - SLAM tracking lost. Dropping frame.");
    return;
  }

  last_publish_time = stamp;
  detectPoseJump(position_body);

  msg_body_pose.header.stamp = msg->header.stamp;
  msg_body_pose.header.frame_id = msg->header.frame_id;
  msg_body_pose.pose.position.x = position_body.getX();
  msg_body_pose.pose.position.y = position_body.getY();
  msg_body_pose.pose.position.z = position_body.getZ();
  msg_body_pose.pose.orientation.x = quat_body.getX();
  msg_body_pose.pose.orientation.y = quat_body.getY();
  msg_body_pose.pose.orientation.z = quat_body.getZ();
  msg_body_pose.pose.orientation.w = quat_body.getW();
  camera_pose_publisher->publish(msg_body_pose);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
  pose_cov_msg.header = msg_body_pose.header;
  pose_cov_msg.pose.pose = msg_body_pose.pose;
  // PoseStamped carries no covariance, so always use the fallback
  pose_cov_msg.pose.covariance = calculatePoseCovariance(3);
  vision_pose_cov_publisher->publish(pose_cov_msg);

  // Velocity from the separate SLAM velocity topic. Skipped if none has
  // arrived yet or the latest one is too far from this pose in time.
  bool velocity_fresh =
      velocity_received &&
      std::abs((stamp - latest_velocity_stamp).seconds()) <= velocity_timeout;
  if (!velocity_fresh) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No fresh velocity on '%s' - publishing pose only.",
                         velocity_topic.c_str());
  } else {
    tf2::Vector3 velocity_odom =
        velocity_in_camera_frame ? tf2::quatRotate(quat_cam, latest_velocity)
                                 : latest_velocity;

    // Same world yaw as the position
    tf2::Vector3 velocity_world;
    velocity_world.setX(cos(gamma_world) * velocity_odom.getX() +
                        sin(gamma_world) * velocity_odom.getY());
    velocity_world.setY(-sin(gamma_world) * velocity_odom.getX() +
                        cos(gamma_world) * velocity_odom.getY());
    velocity_world.setZ(velocity_odom.getZ());

    if (std::isnan(velocity_world.getX()) ||
        std::isnan(velocity_world.getY()) ||
        std::isnan(velocity_world.getZ())) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "NaN in velocity - dropping velocity.");
    } else {
      geometry_msgs::msg::TwistWithCovarianceStamped speed_msg;
      speed_msg.header = msg_body_pose.header;
      speed_msg.twist.twist.linear.x = velocity_world.getX();
      speed_msg.twist.twist.linear.y = velocity_world.getY();
      speed_msg.twist.twist.linear.z = velocity_world.getZ();
      speed_msg.twist.twist.angular.x = 0.0;
      speed_msg.twist.twist.angular.y = 0.0;
      speed_msg.twist.twist.angular.z = 0.0;

      // The velocity topic carries no covariance, so always use the fallback
      speed_msg.twist.covariance = calculateTwistCovariance(3);
      vision_speed_publisher->publish(speed_msg);
    }
  }

  body_path.header = msg_body_pose.header;
  body_path.poses.push_back(msg_body_pose);
  body_path_publisher->publish(body_path);

  tf2::Matrix3x3 m(quat_body);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Pos:[%.2f,%.2f,%.2f] RPY:[%.1f,%.1f,%.1f] Reset:%d",
                       position_body.getX(), position_body.getY(),
                       position_body.getZ(), roll * 180.0 / M_PI,
                       pitch * 180.0 / M_PI, yaw * 180.0 / M_PI, reset_counter);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionToMavros>());
  rclcpp::shutdown();
  return 0;
}
