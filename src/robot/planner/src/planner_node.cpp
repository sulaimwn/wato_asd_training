#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  int obstacle_threshold = this->declare_parameter<int>("obstacle_threshold", 50);
  double cost_weight = this->declare_parameter<double>("cost_weight", 3.0);
  double escape_radius = this->declare_parameter<double>("escape_radius", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter<double>("goal_timeout", 120.0);
  int replan_period_ms = this->declare_parameter<int>("replan_period_ms", 500);

  planner_.setParams(obstacle_threshold, cost_weight, escape_radius);

  // transient_local to match map_memory, so we get the last map even if we start after it
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(), std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(replan_period_ms), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();  // the map changed, so the old path may now go through something
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (have_map_ && !msg->header.frame_id.empty() && msg->header.frame_id != current_map_.header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but the map is in '%s'; treating it as map coordinates",
                msg->header.frame_id.c_str(), current_map_.header.frame_id.c_str());
  }
  goal_ = *msg;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  if ((this->now() - goal_start_time_).seconds() > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Could not reach goal within %.0f s, giving up", goal_timeout_);
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  // Replan from where the robot is now so the path always starts at the robot
  planPath();
}

bool PlannerNode::goalReached() const {
  if (!have_odom_) return false;
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Cannot plan path: missing map or odometry");
    return;
  }

  using Result = robot::PlannerCore::Result;
  nav_msgs::msg::Path path;
  Result result = planner_.planPath(current_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y, path);

  if (result == Result::OK) {
    path.header.stamp = this->now();
    path_pub_->publish(path);
    return;
  }

  publishEmptyPath();  // don't keep following a path we know is bad
  if (result == Result::GOAL_INVALID) {
    // Retrying won't help, so drop the goal
    RCLCPP_WARN(this->get_logger(), "Goal (%.2f, %.2f) is off the map or inside an obstacle, dropping it",
                goal_.point.x, goal_.point.y);
    state_ = State::WAITING_FOR_GOAL;
  } else if (result == Result::START_INVALID) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Robot (%.2f, %.2f) is off the map", robot_x_, robot_y_);
  } else {
    // The map may still open up a way through, so keep retrying
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No path to goal (%.2f, %.2f) yet, retrying", goal_.point.x, goal_.point.y);
  }
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = have_map_ ? current_map_.header.frame_id : "sim_world";
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
