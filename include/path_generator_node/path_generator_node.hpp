// Copyright 2023 ROBOTIS CO., LTD.
// Authors: Sungho Woo

#ifndef PATH_GENERATOR_NODE__PATH_GENERATOR_NODE_HPP_
#define PATH_GENERATOR_NODE__PATH_GENERATOR_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"


struct RobotPose
{
  double x;
  double y;
  double theta;
};

class PathGenerator : public rclcpp::Node
{
public:
  explicit PathGenerator(const std::string node_name);
  virtual ~PathGenerator();

private:
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  void subscribe_odom(const nav_msgs::msg::Odometry::SharedPtr odom_data);
  void subscribe_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_data);
  void publish_path();
  void update_parameter();

  std::shared_ptr<std::mutex> odom_mutex_;
  std::shared_ptr<std::mutex> goal_mutex_;
  std::string frame_id_;
  double aligning_path_m_;
  uint32_t aligning_path_cm_;

  RobotPose robot_pose_;
  RobotPose robot_goal_;
};
#endif  // PATH_GENERATOR_NODE__PATH_GENERATOR_NODE_HPP_
