#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  robot::PlannerCore::Params params;
  params.body_margin = this->declare_parameter<double>("body_margin", params.body_margin);
  params.cost_weight = this->declare_parameter<double>("cost_weight", params.cost_weight);
  params.escape_radius = this->declare_parameter<double>("escape_radius", params.escape_radius);
  params.arrive_radius = this->declare_parameter<double>("arrive_radius", params.arrive_radius);
  params.goal_search_radius = this->declare_parameter<double>("goal_search_radius", params.goal_search_radius);
  arrive_radius_ = params.arrive_radius;
  body_margin_ = params.body_margin;
  switch_distance_ = this->declare_parameter<double>("switch_distance", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter<double>("goal_timeout", 300.0);
  axle_offset_ = this->declare_parameter<double>("axle_offset", 1.3);
  int replan_period_ms = this->declare_parameter<int>("replan_period_ms", 500);

  planner_.setParams(params);

  // transient_local to match map_memory, so we get the last map even if we start after it
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(), std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  // Any message here drops the current goal and stops the robot
  cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/goal_cancel", 10, std::bind(&PlannerNode::cancelCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(replan_period_ms), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.setMap(*msg);
  map_frame_ = msg->header.frame_id;
  have_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();  // the map changed, so the old path may now go through something
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (have_map_ && !msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but the map is in '%s'; treating it as map coordinates",
                msg->header.frame_id.c_str(), map_frame_.c_str());
  }
  goal_ = *msg;
  target_x_ = goal_.point.x;
  target_y_ = goal_.point.y;
  current_path_.poses.clear();
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::cancelCallback(const std_msgs::msg::Empty::SharedPtr) {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
  }
  state_ = State::WAITING_FOR_GOAL;
  publishEmptyPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  robot_x_ = msg->pose.pose.position.x - axle_offset_ * std::cos(robot_yaw_);
  robot_y_ = msg->pose.pose.position.y - axle_offset_ * std::sin(robot_yaw_);
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
  double dx = target_x_ - robot_x_;
  double dy = target_y_ - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Cannot plan path: missing map or odometry");
    return;
  }

  using Result = robot::PlannerCore::Result;
  nav_msgs::msg::Path path;
  // Plan to the goal, unless it's already turned out the robot can't get
  // there: then to the nearest spot it can, found last time, which is much
  // quicker than searching everywhere for a way to the goal again
  const bool detour = std::hypot(target_x_ - goal_.point.x, target_y_ - goal_.point.y) > arrive_radius_ + 0.05;
  double end_x = detour ? target_x_ : goal_.point.x;
  double end_y = detour ? target_y_ : goal_.point.y;
  Result result = planner_.planPath(robot_x_, robot_y_, robot_yaw_, end_x, end_y, path);

  if (result == Result::OK) {
    if (!detour) {
      if (std::hypot(end_x - goal_.point.x, end_y - goal_.point.y) > arrive_radius_ + 0.05) {
        RCLCPP_INFO(this->get_logger(), "The robot can't get to (%.2f, %.2f), going as close as it can, (%.2f, %.2f)",
                    goal_.point.x, goal_.point.y, end_x, end_y);
      }
      target_x_ = end_x;
      target_y_ = end_y;
    }
    // Stay on the route we're already driving unless the new one is clearly
    // better: where two ways round cost about the same, each replan from a
    // slightly different spot can pick the other, and the robot turns back
    // and forth between them
    if (!keepCurrentPath(path)) current_path_ = path;
    current_path_.header.stamp = this->now();
    path_pub_->publish(current_path_);
    return;
  }

  publishEmptyPath();  // don't keep following a path we know is bad
  if (result == Result::GOAL_INVALID) {
    // Retrying won't help, so drop the goal
    RCLCPP_WARN(this->get_logger(), "Goal (%.2f, %.2f) is off the map or the robot can't get near it, dropping it",
                goal_.point.x, goal_.point.y);
    state_ = State::WAITING_FOR_GOAL;
  } else if (result == Result::START_INVALID) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Robot (%.2f, %.2f) is off the map", robot_x_, robot_y_);
  } else {
    // Hemmed in: that may not last (the map changes, the robot backs up), so keep retrying
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No path to goal (%.2f, %.2f) yet, retrying", goal_.point.x, goal_.point.y);
  }
}

// True if the path we're on is still worth following: it leads to the same
// place, the robot is still on it, the body still clears everything along
// the rest of it on the latest map, and the new path isn't at least
// switch_distance shorter. Then current_path_ is trimmed to start at the robot.
bool PlannerNode::keepCurrentPath(const nav_msgs::msg::Path& fresh) {
  const auto& old_poses = current_path_.poses;
  if (old_poses.empty() || fresh.poses.empty()) return false;
  const auto& old_end = old_poses.back().pose.position;
  const auto& new_end = fresh.poses.back().pose.position;
  if (std::hypot(old_end.x - new_end.x, old_end.y - new_end.y) > 0.3) return false;

  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < old_poses.size(); ++i) {
    const double d = std::hypot(old_poses[i].pose.position.x - robot_x_, old_poses[i].pose.position.y - robot_y_);
    if (d < best) {
      best = d;
      closest = i;
    }
  }
  if (best > 0.5) return false;  // wandered off it
  if (planner_.pathClearance(current_path_, closest) < body_margin_ - 0.05) return false;  // something's in the way now

  auto length = [](const std::vector<geometry_msgs::msg::PoseStamped>& poses, size_t from) {
    double total = 0.0;
    for (size_t i = from + 1; i < poses.size(); ++i) {
      total += std::hypot(poses[i].pose.position.x - poses[i - 1].pose.position.x,
                          poses[i].pose.position.y - poses[i - 1].pose.position.y);
    }
    return total;
  };
  if (length(fresh.poses, 0) < best + length(old_poses, closest) - switch_distance_) return false;

  current_path_.poses.erase(current_path_.poses.begin(), current_path_.poses.begin() + closest);
  return true;
}

void PlannerNode::publishEmptyPath() {
  current_path_.poses.clear();
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_frame_;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
