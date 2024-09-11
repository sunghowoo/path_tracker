// Copyright 2023 ROBOTIS CO., LTD.
// Authors: Sungho Woo

#include <path_generator_node/path_generator_node.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <memory>


PathGenerator::PathGenerator(const std::string node_name)
: Node(node_name)
{
  this->declare_parameter("aligning_path_length", 0.5);
  this->declare_parameter("qos_depth", 10);
  this->update_parameter();

  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  aligning_path_m_ = this->get_parameter("aligning_path_length").get_value<double>();

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", QOS_RKL10V);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", QOS_RKL10V, std::bind(&PathGenerator::subscribe_odom, this, std::placeholders::_1));
  goalpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", QOS_RKL10V, std::bind(
      &PathGenerator::subscribe_goal,
      this, std::placeholders::_1));
  path_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.05),
    std::bind(&PathGenerator::publish_path, this));

  frame_id_ = "odom";
  aligning_path_cm_ = static_cast<uint32_t>(aligning_path_m_ * 100);
  robot_pose_ = {0.0, 0.0, 0.0};
  robot_goal_ = {0.0, 0.0, 0.0};
  odom_mutex_ = std::make_shared<std::mutex>();
  goal_mutex_ = std::make_shared<std::mutex>();
}

PathGenerator::~PathGenerator()
{
}

void PathGenerator::update_parameter()
{
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

  constexpr uint8_t MAX_ATTEMPTS = 10;
  uint8_t attempts = 0;
  while (attempts < MAX_ATTEMPTS) {
    if (parameters_client_->wait_for_service(std::chrono::seconds(1))) {
      break;
    }
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    attempts++;
  }

  if (attempts == MAX_ATTEMPTS) {
    RCLCPP_ERROR(this->get_logger(), "Service not available after multiple attempts. Exiting.");
    return;
  }

  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
      for (const auto & changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "aligning_path_length") {
          auto param_value = this->get_parameter("aligning_path_length");
          if (param_value.get_type() == rclcpp::PARAMETER_DOUBLE) {
            aligning_path_m_ = param_value.get_value<double>();
            aligning_path_cm_ = static_cast<uint32_t>(aligning_path_m_ * 100);
            RCLCPP_INFO(this->get_logger(), "Updated aligning_path_m_ to: %f", aligning_path_m_);
          } else {
            RCLCPP_WARN(this->get_logger(), "Parameter aligning path_length is not a double.");
          }
        }
      }
    };

  parameter_event_sub_ = parameters_client_->on_parameter_event(param_event_callback);
}

void PathGenerator::subscribe_odom(const nav_msgs::msg::Odometry::SharedPtr odom_data)
{
  geometry_msgs::msg::Quaternion orientation = odom_data->pose.pose.orientation;
  geometry_msgs::msg::Point position = odom_data->pose.pose.position;

  tf2::Quaternion tf_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3 tf_matrix(tf_orientation);
  double roll, pitch, yaw;
  tf_matrix.getRPY(roll, pitch, yaw);

  {
    std::lock_guard<std::mutex> lock(*odom_mutex_);
    robot_pose_ = {position.x, position.y, yaw};
  }
}

void PathGenerator::subscribe_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_data)
{
  geometry_msgs::msg::Quaternion orientation = goal_data->pose.orientation;
  geometry_msgs::msg::Point position = goal_data->pose.position;

  tf2::Quaternion tf_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3 tf_matrix(tf_orientation);
  double roll, pitch, yaw;
  tf_matrix.getRPY(roll, pitch, yaw);
  {
    std::lock_guard<std::mutex> lock(*goal_mutex_);
    robot_goal_ = {position.x, position.y, yaw};
  }
}

void PathGenerator::publish_path()
{
  double robot_x, robot_y, goal_x, goal_y, goal_theta;

  {
    std::lock_guard<std::mutex> lock(*odom_mutex_);
    robot_x = robot_pose_.x;
    robot_y = robot_pose_.y;
  }

  {
    std::lock_guard<std::mutex> lock(*goal_mutex_);
    goal_x = robot_goal_.x;
    goal_y = robot_goal_.y;
    goal_theta = robot_goal_.theta;
  }

  if (goal_x != 0) {
    double virtual_goalx = goal_x - (aligning_path_cm_ * 0.01 * std::cos(goal_theta));
    double virtual_goaly = goal_y - (aligning_path_cm_ * 0.01 * std::sin(goal_theta));

    double distance_between_robot_and_virtual_goal = std::sqrt(
      std::pow((robot_x - virtual_goalx), 2) +
      std::pow((robot_y - virtual_goaly), 2));
    double n_steps = distance_between_robot_and_virtual_goal / 0.01;
    double alpha = std::atan2((virtual_goaly - robot_y), (virtual_goalx - robot_x));

    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<geometry_msgs::msg::PoseStamped> poses;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom";

    for (uint32_t i = 0; i < static_cast<uint32_t>(n_steps); ++i) {
      double x = robot_x + i * (0.01 * std::cos(alpha));
      double y = robot_y + i * (0.01 * std::sin(alpha));
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      poses.push_back(pose);
    }

    constexpr uint8_t PATH_OFFSET = 50;
    for (uint32_t i = 0; i < static_cast<uint32_t>(aligning_path_cm_) + PATH_OFFSET; ++i) {
      double x = virtual_goalx + i * (0.01 * std::cos(goal_theta));
      double y = virtual_goaly + i * (0.01 * std::sin(goal_theta));
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      poses.push_back(pose);
    }

    path_msg.poses = poses;
    path_pub_->publish(path_msg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathGenerator>("path_generator_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
