#include <chrono>
#include <cmath>
#include <memory>

#include "explorer_node.hpp"

ExplorerNode::ExplorerNode() : Node("explorer"), explorer_(robot::ExplorerCore(this->get_logger())) {
  robot::ExplorerCore::Params params;
  params.free_threshold = this->declare_parameter<int>("free_threshold", params.free_threshold);
  params.frontier_max_cost = this->declare_parameter<int>("frontier_max_cost", params.frontier_max_cost);
  params.min_frontier_size = this->declare_parameter<int>("min_frontier_size", params.min_frontier_size);
  params.escape_radius = this->declare_parameter<double>("escape_radius", params.escape_radius);
  explorer_.setParams(params);

  goal_reached_dist_ = this->declare_parameter<double>("goal_reached_dist", 1.0);
  frontier_resolved_radius_ = this->declare_parameter<double>("frontier_resolved_radius", 1.0);
  blacklist_radius_ = this->declare_parameter<double>("blacklist_radius", 1.5);
  stuck_timeout_ = this->declare_parameter<double>("stuck_timeout", 20.0);
  path_fail_timeout_ = this->declare_parameter<double>("path_fail_timeout", 4.0);
  return_home_ = this->declare_parameter<bool>("return_home", true);
  bool start_enabled = this->declare_parameter<bool>("start_enabled", false);
  int update_period_ms = this->declare_parameter<int>("update_period_ms", 1000);

  // transient_local to match map_memory, so we get the last map even if we start after it
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(), std::bind(&ExplorerNode::mapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ExplorerNode::odomCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ExplorerNode::pathCallback, this, std::placeholders::_1));
  enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/explore/enable", 10, std::bind(&ExplorerNode::enableCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&ExplorerNode::goalCallback, this, std::placeholders::_1));

  goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 10);
  cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>("/goal_cancel", 10);
  // transient_local so a Foxglove panel opened later still shows the current state
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/explore/status", rclcpp::QoS(1).transient_local());
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/explore/frontiers", 10);

  last_progress_time_ = last_path_time_ = this->now();
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(update_period_ms), std::bind(&ExplorerNode::tick, this));

  if (start_enabled) {
    start();
  } else {
    setState(State::IDLE, "publish true on /explore/enable to start");
  }
}

void ExplorerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  have_map_ = true;
}

void ExplorerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
  if (state_ == State::EXPLORING && !have_home_) {
    home_x_ = robot_x_;
    home_y_ = robot_y_;
    have_home_ = true;
  }
}

void ExplorerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  // The planner sends an empty path when it can't find a way (or is stopping)
  if (msg->poses.empty()) return;
  last_path_time_ = this->now();

  // The route starts at the robot, so its length is the distance still to go
  double length = 0.0;
  for (size_t i = 1; i < msg->poses.size(); ++i) {
    const auto& a = msg->poses[i - 1].pose.position;
    const auto& b = msg->poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  path_length_ = length;
}

void ExplorerNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    if (state_ != State::EXPLORING && state_ != State::RETURNING_HOME) start();
  } else if (state_ != State::IDLE) {
    if (state_ != State::COMPLETE) cancelGoal();
    setState(State::IDLE, "turned off");
  }
}

// A goal we didn't send means someone clicked one in Foxglove: let them drive
void ExplorerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (state_ != State::EXPLORING && state_ != State::RETURNING_HOME) return;
  bool ours = have_goal_ && std::abs(msg->point.x - goal_x_) < 1e-6 && std::abs(msg->point.y - goal_y_) < 1e-6;
  if (ours) return;
  have_goal_ = false;
  setState(State::IDLE, "a goal was sent from elsewhere, handing over control");
}

void ExplorerNode::start() {
  have_home_ = false;
  if (have_odom_) {
    home_x_ = robot_x_;
    home_y_ = robot_y_;
    have_home_ = true;
  }
  blacklist_.clear();
  rounds_without_frontier_ = 0;
  have_goal_ = false;
  setState(State::EXPLORING, "turned on");
}

void ExplorerNode::tick() {
  if (state_ != State::EXPLORING && state_ != State::RETURNING_HOME) return;
  if (!have_map_ || !have_odom_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for the map and odometry");
    return;
  }

  std::vector<std::pair<double, double>> cells;
  std::vector<robot::Frontier> frontiers = explorer_.findFrontiers(map_, robot_x_, robot_y_, &cells);
  publishMarkers(cells);

  // How we're doing on the current goal. Progress is measured along the
  // planner's route, not in a straight line: reaching the far side of a wall
  // can mean driving away from the goal for a while.
  const rclcpp::Time now = this->now();
  double goal_dist = std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_);
  double remaining = path_length_ >= 0.0 ? path_length_ : goal_dist;
  if (have_goal_ && (remaining < best_remaining_ - 0.3 || remaining > best_remaining_ + 1.0)) {
    // Closer, or the planner found the old route blocked and picked a longer one
    best_remaining_ = remaining;
    last_progress_time_ = now;
  }
  bool reached = have_goal_ && goal_dist < goal_reached_dist_;
  bool no_path = have_goal_ && (now - last_path_time_).seconds() > path_fail_timeout_;
  bool stuck = have_goal_ && (now - last_progress_time_).seconds() > stuck_timeout_;

  if (state_ == State::RETURNING_HOME) {
    if (reached || no_path || stuck) {
      cancelGoal();
      setState(State::COMPLETE, reached ? "back home, map complete" : "map complete, but couldn't get back home");
    }
    return;
  }

  // Exploring: move on from the current frontier once it is dealt with
  if (have_goal_) {
    const char* why = nullptr;
    if (reached) {
      why = "reached";
    } else if (!robot::ExplorerCore::nearAny(cells, goal_x_, goal_y_, frontier_resolved_radius_)) {
      why = "explored before we got there";  // the lidar has already seen it
    } else if (no_path) {
      why = "planner found no path";
    } else if (stuck) {
      why = "not getting closer";
    }
    if (why) {
      RCLCPP_INFO(this->get_logger(), "Frontier (%.1f, %.1f): %s", goal_x_, goal_y_, why);
      if (reached || no_path || stuck) blacklist_.emplace_back(goal_x_, goal_y_);
      have_goal_ = false;
    }
  }
  if (have_goal_) return;

  if (pickNextFrontier(frontiers)) {
    rounds_without_frontier_ = 0;
    return;
  }

  // Nothing left. Give the map a couple of updates to catch up before finishing.
  if (++rounds_without_frontier_ < 3) return;
  if (return_home_ && have_home_ && std::hypot(home_x_ - robot_x_, home_y_ - robot_y_) > goal_reached_dist_) {
    sendGoal(home_x_, home_y_);
    setState(State::RETURNING_HOME, "no frontiers left");
  } else {
    cancelGoal();
    setState(State::COMPLETE, "no frontiers left");
  }
}

// Closest frontier that isn't next to a goal we've already reached or given up on
bool ExplorerNode::pickNextFrontier(const std::vector<robot::Frontier>& frontiers) {
  for (const auto& f : frontiers) {
    if (blacklisted(f.goal_x, f.goal_y)) continue;
    if (std::hypot(f.goal_x - robot_x_, f.goal_y - robot_y_) < goal_reached_dist_) {
      // We're standing on it and the map still says it's unexplored, so
      // driving to it again won't help
      blacklist_.emplace_back(f.goal_x, f.goal_y);
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "Heading to frontier (%.1f, %.1f): %d cells, %.1f m away",
                f.goal_x, f.goal_y, f.size, f.distance);
    sendGoal(f.goal_x, f.goal_y);
    return true;
  }
  return false;
}

bool ExplorerNode::blacklisted(double x, double y) const {
  return robot::ExplorerCore::nearAny(blacklist_, x, y, blacklist_radius_);
}

void ExplorerNode::sendGoal(double x, double y) {
  geometry_msgs::msg::PointStamped goal;
  goal.header.stamp = this->now();
  goal.header.frame_id = map_.header.frame_id.empty() ? "sim_world" : map_.header.frame_id;
  goal.point.x = x;
  goal.point.y = y;

  have_goal_ = true;
  goal_x_ = x;
  goal_y_ = y;
  path_length_ = -1.0;  // until the planner sends a route for this goal
  best_remaining_ = std::hypot(x - robot_x_, y - robot_y_);
  last_progress_time_ = last_path_time_ = this->now();
  goal_pub_->publish(goal);
}

void ExplorerNode::cancelGoal() {
  have_goal_ = false;
  cancel_pub_->publish(std_msgs::msg::Empty());
}

void ExplorerNode::setState(State state, const std::string& why) {
  state_ = state;
  RCLCPP_INFO(this->get_logger(), "%s: %s", stateName(state), why.c_str());

  std_msgs::msg::String msg;
  msg.data = stateName(state);
  status_pub_->publish(msg);

  if (state == State::IDLE || state == State::COMPLETE) publishMarkers({});
}

// Frontier cells (cyan), the current goal (pink) and home (green) for Foxglove
void ExplorerNode::publishMarkers(const std::vector<std::pair<double, double>>& cells) {
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  const std::string frame = map_.header.frame_id.empty() ? "sim_world" : map_.header.frame_id;
  const auto stamp = this->now();

  auto make = [&](const std::string& ns, int type, bool show) {
    Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = 0;
    m.type = type;
    m.action = show ? Marker::ADD : Marker::DELETE;
    m.pose.orientation.w = 1.0;
    return m;
  };

  Marker frontier = make("frontiers", Marker::CUBE_LIST, !cells.empty());
  double res = have_map_ ? map_.info.resolution : 0.1;
  frontier.scale.x = frontier.scale.y = res;
  frontier.scale.z = 0.05;
  frontier.color.r = 0.0;
  frontier.color.g = 0.9;
  frontier.color.b = 1.0;
  frontier.color.a = 0.9;
  for (const auto& c : cells) {
    geometry_msgs::msg::Point p;
    p.x = c.first;
    p.y = c.second;
    p.z = 0.05;
    frontier.points.push_back(p);
  }
  markers.markers.push_back(frontier);

  bool driving = have_goal_ && (state_ == State::EXPLORING || state_ == State::RETURNING_HOME);
  Marker goal = make("goal", Marker::SPHERE, driving);
  goal.pose.position.x = goal_x_;
  goal.pose.position.y = goal_y_;
  goal.pose.position.z = 0.4;
  goal.scale.x = goal.scale.y = goal.scale.z = 0.7;
  goal.color.r = 1.0;
  goal.color.g = 0.2;
  goal.color.b = 0.8;
  goal.color.a = 0.9;
  markers.markers.push_back(goal);

  Marker home = make("home", Marker::CYLINDER, have_home_);
  home.pose.position.x = home_x_;
  home.pose.position.y = home_y_;
  home.pose.position.z = 0.05;
  home.scale.x = home.scale.y = 0.8;
  home.scale.z = 0.1;
  home.color.r = 0.2;
  home.color.g = 0.9;
  home.color.b = 0.3;
  home.color.a = 0.8;
  markers.markers.push_back(home);

  marker_pub_->publish(markers);
}

const char* ExplorerNode::stateName(State state) {
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::EXPLORING: return "EXPLORING";
    case State::RETURNING_HOME: return "RETURNING_HOME";
    case State::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
