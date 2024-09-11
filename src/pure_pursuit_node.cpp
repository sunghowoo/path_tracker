// Copyright 2023 ROBOTIS CO., LTD.
// Authors: Sungho Woo

#include <pure_pursuit_node/pure_pursuit_node.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <memory>


PurePursuit::PurePursuit(const std::string node_name)
: Node(node_name)
{
  this->declare_parameter("p_gain", 5.0);
  this->declare_parameter("d_gain", 5.0);
  this->declare_parameter("lookahead_distance", 0.2);
  this->declare_parameter("max_linear_speed", 0.2);
  this->declare_parameter("max_angular_speed", 2.83);
  this->declare_parameter("qos_depth", 10);
  this->update_parameter();

  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  p_gain_ = this->get_parameter("p_gain").get_value<double>();
  d_gain_ = this->get_parameter("d_gain").get_value<double>();
  lookahead_distance_ = this->get_parameter("lookahead_distance").get_value<double>();
  max_linear_speed_ = this->get_parameter("max_linear_speed").get_value<double>();
  max_angular_speed_ = this->get_parameter("max_angular_speed").get_value<double>();

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", QOS_RKL10V);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead", QOS_RKL10V);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", QOS_RKL10V, std::bind(&PurePursuit::subscribe_odom, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", QOS_RKL10V, std::bind(&PurePursuit::subscribe_path, this, std::placeholders::_1));
  path_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.05), std::bind(&PurePursuit::publish_cmd_velocity, this));
  docking_server_ = this->create_service<std_srvs::srv::SetBool>(
    "docking_activate",
    std::bind(
      &PurePursuit::subscribe_docking_activation_service, this,
      std::placeholders::_1, std::placeholders::_2));

  odom_mutex_ = std::make_shared<std::mutex>();
  path_mutex_ = std::make_shared<std::mutex>();
  last_error_ = 0.0;
  frame_id_ = "odom";
  robot_pose_ = {0.0, 0.0, 0.0};
  activation_flag_ = false;
  pd_scale_ = 1.0;
}

PurePursuit::~PurePursuit()
{
}

void PurePursuit::update_parameter()
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
        if (changed_parameter.name == "p_gain") {
          p_gain_ = this->get_parameter("p_gain").get_value<double>();
        } else if (changed_parameter.name == "d_gain") {
          d_gain_ = this->get_parameter("d_gain").get_value<double>();
        } else if (changed_parameter.name == "lookahead_distance") {
          lookahead_distance_ = this->get_parameter("lookahead_distance").get_value<double>();
        } else if (changed_parameter.name == "max_linear_speed") {
          max_linear_speed_ = this->get_parameter("max_linear_speed").get_value<double>();
        } else if (changed_parameter.name == "max_angular_speed") {
          max_angular_speed_ = this->get_parameter("max_angular_speed").get_value<double>();
        }
      }
    };

  parameter_event_sub_ = parameters_client_->on_parameter_event(param_event_callback);
}

void PurePursuit::subscribe_docking_activation_service(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  activation_flag_ = request->data;

  RCLCPP_INFO(
    this->get_logger(), "Docking activation request received: %s",
    request->data ? "true" : "false");

  response->success = true;
  response->message = "Docking activation set to " +
    std::string(request->data ? "true" : "false");

  RCLCPP_INFO(
    this->get_logger(), "Docking activation response sent: %s",
    response->success ? "success" : "failed");
}

void PurePursuit::draw_a_point(
  const std::string & ns, uint16_t id, double px, double py, Color color,
  rclcpp::Time now)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now;
  marker.header.frame_id = frame_id_;
  marker.ns = ns;
  marker.id = id;
  marker.type = marker.SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = (color == Color::RED) ? 1.0 : 0.0;
  marker.color.g = (color == Color::GREEN) ? 1.0 : 0.0;
  marker.color.b = (color == Color::BLUE) ? 1.0 : 0.0;
  marker.pose.position.x = px;
  marker.pose.position.y = py;
  marker.pose.position.z = 0.01;
  marker_pub_->publish(marker);
}

void PurePursuit::subscribe_odom(const nav_msgs::msg::Odometry::SharedPtr odom_data)
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

void PurePursuit::subscribe_path(const nav_msgs::msg::Path::SharedPtr path_data)
{
  {
    std::lock_guard<std::mutex> lock(*path_mutex_);
    origin_path_ = *path_data;
  }
}

double PurePursuit::calculate_control_error(double delta_theta)
{
  double derivative = delta_theta - last_error_;
  last_error_ = delta_theta;
  return (delta_theta * p_gain_) + (d_gain_ * derivative);
}

auto PurePursuit::calculate_velocity(
  const geometry_msgs::msg::Quaternion & lookahead_position, const RobotPose & robot_pose,
  double goal_x, double goal_y)
{
  geometry_msgs::msg::Twist vel;
  double robot_x = robot_pose.x;
  double robot_y = robot_pose.y;
  double robot_theta = robot_pose.theta;
  double delta_x = (lookahead_position.x - robot_x) * std::cos(robot_theta) +
    (lookahead_position.y - robot_y) * std::sin(robot_theta);
  double delta_y = -(lookahead_position.x - robot_x) * std::sin(robot_theta) +
    (lookahead_position.y - robot_y) * std::cos(robot_theta);
  double delta_theta_radian = std::atan2(delta_y, delta_x);
  double pd_delta_theta = calculate_control_error(delta_theta_radian);
  double delta_theta_degree = std::round(delta_theta_radian * (180.0 / M_PI));
  constexpr double PATH_OFFSET = 0.5;
  double distance_from_end = std::hypot(goal_x - robot_x, goal_y - robot_y);
  double real_end_distance = distance_from_end - PATH_OFFSET;


  constexpr double MAX_HEADING_ERROR = 60.0;
  if (std::abs(delta_theta_degree) > MAX_HEADING_ERROR) {
    vel.linear.x = 0.0;
    vel.angular.z = pd_scale_ * pd_delta_theta;
  } else {
    vel.linear.x = max_linear_speed_;
    vel.angular.z = pd_scale_ * pd_delta_theta;
  }

  if (distance_from_end <= PATH_OFFSET || activation_flag_ == false) {
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    activation_flag_ = false;
  } else if (real_end_distance <= 0.5) {
    vel.linear.x = 0.1 / 0.45 * (real_end_distance - 0.05) + 0.1;
  } else if (real_end_distance <= 0.1) {
    vel.linear.x = 0.1;
  }

  if (vel.angular.z > max_angular_speed_) {
    vel.angular.z = max_angular_speed_;
  } else if (vel.angular.z < -max_angular_speed_) {
    vel.angular.z = -max_angular_speed_;
  }
  return vel;
}

void PurePursuit::publish_cmd_velocity()
{
  RobotPose robot_pose;
  {
    std::lock_guard<std::mutex> lock(*odom_mutex_);
    robot_pose = robot_pose_;
  }

  auto poses = origin_path_.poses;

  if (poses.size() > 0) {
    int32_t point_index = -1;
    double goal_x = poses.back().pose.position.x;
    double goal_y = poses.back().pose.position.y;

    for (int32_t i = poses.size() - 1; i >= 0; --i) {
      double point_x = poses[i].pose.position.x;
      double point_y = poses[i].pose.position.y;
      double distance_from_lookahead = std::sqrt(
        std::pow((robot_pose.x - point_x), 2) +
        std::pow((robot_pose.y - point_y), 2));

      if (distance_from_lookahead < lookahead_distance_) {
        point_index = i;
        break;
      }
    }

    double lookahead_x = poses[point_index].pose.position.x;
    double lookahead_y = poses[point_index].pose.position.y;
    draw_a_point("lookahead", 3, lookahead_x, lookahead_y, Color::BLUE, this->now());

    geometry_msgs::msg::Quaternion lookahead_position;
    lookahead_position.x = lookahead_x;
    lookahead_position.y = lookahead_y;

    auto cmd = calculate_velocity(lookahead_position, robot_pose, goal_x, goal_y);
    cmd_pub_->publish(cmd);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuit>("pure_pursuit_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
