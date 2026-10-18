#pragma once
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mavros_msgs/msg/landing_target.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Converts the ORB-SLAM3 node's pose (geometry_msgs/PoseStamped) and velocity
// (geometry_msgs/TwistStamped) topics into MAVROS vision pose / speed messages.
class VisionToMavros : public rclcpp::Node {
 public:
  VisionToMavros();
  ~VisionToMavros() {}

 private:
  void navigationParameters(void);
  void precisionLandParameters(void);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void detectPoseJump(const tf2::Vector3 &current_pos);
  std::array<double, 36> calculatePoseCovariance(int confidence_level);
  std::array<double, 36> calculateTwistCovariance(int confidence_level);

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      camera_pose_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_path_publisher;
  rclcpp::Publisher<mavros_msgs::msg::LandingTarget>::SharedPtr
      precland_msg_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vision_pose_cov_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      vision_speed_publisher;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      velocity_subscriber;

  // Latest translational velocity from the SLAM velocity topic
  tf2::Vector3 latest_velocity;
  rclcpp::Time latest_velocity_stamp;
  bool velocity_received;

  // Transform variables
  tf2::Vector3 position_orig, position_body;
  tf2::Quaternion quat_cam, quat_cam_to_body, quat_rot_z, quat_body;

  // Messages
  geometry_msgs::msg::PoseStamped msg_body_pose;
  nav_msgs::msg::Path body_path;

  // Rate limiting (based on message stamps)
  rclcpp::Time last_publish_time;

  // Frame IDs
  std::string precland_target_frame_id;
  std::string precland_camera_frame_id;
  std::string pose_topic;
  std::string velocity_topic;

  // Parameters
  double output_rate;
  double gamma_world;
  double roll_cam;
  double pitch_cam;
  double yaw_cam;
  bool enable_precland;
  bool velocity_in_camera_frame;  // false: velocity is already in odom frame
  double velocity_timeout;        // max stamp gap between pose and velocity [s]

  // Covariance and tracking parameters
  double linear_accel_cov;
  double angular_vel_cov;
  double pose_jump_threshold;

  // Tracking variables
  tf2::Vector3 prev_position_body;
  bool first_pose_received;
  uint8_t reset_counter;
};
