#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  robot::ControlCore::Params params;
  params.lookahead_distance = this->declare_parameter<double>("lookahead_distance", params.lookahead_distance);
  params.linear_speed = this->declare_parameter<double>("linear_speed", params.linear_speed);
  params.max_angular_speed = this->declare_parameter<double>("max_angular_speed", params.max_angular_speed);
  params.goal_tolerance = this->declare_parameter<double>("goal_tolerance", params.goal_tolerance);
  params.slowdown_distance = this->declare_parameter<double>("slowdown_distance", params.slowdown_distance);
  params.rotate_in_place_angle = this->declare_parameter<double>("rotate_in_place_angle", params.rotate_in_place_angle);
  params.axle_offset = this->declare_parameter<double>("axle_offset", params.axle_offset);
  control_.setParams(params);

  odom_timeout_ = this->declare_parameter<double>("odom_timeout", 0.5);
  int control_period_ms = this->declare_parameter<int>("control_period_ms", 100);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = msg;
        last_odom_time_ = this->now();
      });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Nothing to follow (no path yet, or the planner sent an empty one to stop us)
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    stop();
    return;
  }

  // Don't drive blind on old odometry
  if ((this->now() - last_odom_time_).seconds() > odom_timeout_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odometry is stale, stopping");
    stop();
    return;
  }

  const auto& pose = robot_odom_->pose.pose;
  const auto& q = pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  auto cmd = control_.computeCommand(*current_path_, pose.position.x, pose.position.y, yaw);
  if (!cmd) {
    stop();  // reached the end of the path
    return;
  }

  cmd_vel_pub_->publish(*cmd);
  driving_ = true;
}

// Send one zero command when we stop, then stay quiet so the teleop panel
// can still drive the robot while we have nothing to do
void ControlNode::stop() {
  if (!driving_) return;
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  driving_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
