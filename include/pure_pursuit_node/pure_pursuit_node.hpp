// Copyright 2023 ROBOTIS CO., LTD.
// Authors: Sungho Woo

#ifndef PURE_PURSUIT_NODE__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT_NODE__PURE_PURSUIT_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"


enum class Color
{
  RED,
  GREEN,
  BLUE
};

struct RobotPose
{
  double x;
  double y;
  double theta;
};

class PurePursuit : public rclcpp::Node
{
public:
  explicit PurePursuit(const std::string node_name);
  virtual ~PurePursuit();

private:
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr docking_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  void draw_a_point(
    const std::string & ns, uint16_t id, double px, double py, Color color,
    rclcpp::Time now);
  void subscribe_odom(const nav_msgs::msg::Odometry::SharedPtr odom_data);
  void subscribe_path(const nav_msgs::msg::Path::SharedPtr path_data);
  double calculate_control_error(double delta_theta);
  auto calculate_velocity(
    const geometry_msgs::msg::Quaternion & lookahead_position,
    const RobotPose & robot_pose, double goal_x, double goal_y);
  void publish_cmd_velocity();
  void subscribe_docking_activation_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void update_parameter();

  std::shared_ptr<std::mutex> odom_mutex_;
  std::shared_ptr<std::mutex> path_mutex_;
  std::string frame_id_;
  nav_msgs::msg::Path origin_path_;

  RobotPose robot_pose_;

  double last_error_;
  double p_gain_;
  double d_gain_;
  double lookahead_distance_;
  double max_linear_speed_;
  double max_angular_speed_;
  double pd_scale_;
  bool activation_flag_;
};
#endif  // PURE_PURSUIT_NODE__PURE_PURSUIT_NODE_HPP_
