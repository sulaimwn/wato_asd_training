#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // 40m x 40m map at 0.1 m/cell, centered on the world origin (the sim arena is 30m x 30m)
  double resolution = this->declare_parameter<double>("resolution", 0.1);
  int width  = this->declare_parameter<int>("width", 400);
  int height = this->declare_parameter<int>("height", 400);
  double origin_x = this->declare_parameter<double>("origin_x", -20.0);
  double origin_y = this->declare_parameter<double>("origin_y", -20.0);
  std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.5);
  int update_period_ms = this->declare_parameter<int>("update_period_ms", 1000);

  map_memory_.initMap(resolution, width, height, origin_x, origin_y, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // transient_local: nodes that start later (e.g. the planner) still get the last map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(update_period_ms), std::bind(&MapMemoryNode::updateMap, this));

  // Publish the (empty) map right away so the planner has something to plan on
  publishMap();
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // The sim's first few scans after startup come back empty. A costmap with no
  // obstacles only turns unknown cells into free ones, which the planner treats
  // the same, so skip it. Otherwise a blank scan uses up the first map update
  // and the next one is 1.5 m of driving away, possibly into what we didn't map.
  bool has_obstacle = std::any_of(msg->data.begin(), msg->data.end(), [](int8_t v) { return v > 0; });
  if (!has_obstacle) return;

  // Pair the costmap with where the robot was when the scan was taken, not
  // where it is when the timer fires; otherwise obstacles smear while turning
  Pose2D pose;
  if (!poseAt(rclcpp::Time(msg->header.stamp).seconds(), pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No odometry close to the costmap timestamp, skipping costmap");
    return;
  }
  latest_costmap_ = *msg;
  costmap_pose_ = pose;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Pose2D pose;
  pose.t = rclcpp::Time(msg->header.stamp).seconds();
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;

  // Quaternion -> yaw (rotation about z)
  const auto& q = msg->pose.pose.orientation;
  pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Drop history if time jumped backwards (e.g. the sim was restarted)
  if (!odom_history_.empty() && pose.t < odom_history_.back().t) odom_history_.clear();

  odom_history_.push_back(pose);
  while (odom_history_.size() > 50) odom_history_.pop_front();  // ~5 s at 10 Hz
}

// Robot pose at time t, interpolated between the two odometry messages around it
bool MapMemoryNode::poseAt(double t, Pose2D& pose) const {
  const double tolerance = 0.2;  // s, how far outside the history we accept
  if (odom_history_.empty()) return false;

  const Pose2D& oldest = odom_history_.front();
  const Pose2D& newest = odom_history_.back();
  if (t <= oldest.t) {
    pose = oldest;
    return oldest.t - t <= tolerance;
  }
  if (t >= newest.t) {
    pose = newest;
    return t - newest.t <= tolerance;
  }

  for (size_t i = 1; i < odom_history_.size(); ++i) {
    const Pose2D& a = odom_history_[i - 1];
    const Pose2D& b = odom_history_[i];
    if (b.t < t) continue;

    double f = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
    double dyaw = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));  // shortest way round
    pose.t = t;
    pose.x = a.x + f * (b.x - a.x);
    pose.y = a.y + f * (b.y - a.y);
    pose.yaw = a.yaw + f * dyaw;
    return true;
  }
  return false;
}

// Timer-based map update: only fuse when the robot has moved far enough
void MapMemoryNode::updateMap() {
  if (!have_costmap_) return;

  const Pose2D& p = costmap_pose_;
  double moved = std::hypot(p.x - last_update_x_, p.y - last_update_y_);
  if (first_update_done_ && moved < distance_threshold_) return;

  map_memory_.integrateCostmap(latest_costmap_, p.x, p.y, p.yaw);
  last_update_x_ = p.x;
  last_update_y_ = p.y;
  first_update_done_ = true;

  publishMap();
}

void MapMemoryNode::publishMap() {
  nav_msgs::msg::OccupancyGrid map_msg = map_memory_.getMap();
  map_msg.header.stamp = this->get_clock()->now();
  map_pub_->publish(map_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
