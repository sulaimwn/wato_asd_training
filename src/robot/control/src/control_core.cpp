#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeCommand(const nav_msgs::msg::Path& path,
                                                                     double x, double y, double yaw) {
  if (path.poses.empty()) return std::nullopt;

  const auto& end = path.poses.back().pose.position;
  double dist_to_end = std::hypot(end.x - x, end.y - y);
  if (dist_to_end < params_.goal_tolerance) return std::nullopt;

  // Closest path point to the robot, then the first point after it that is at
  // least one lookahead away. Searching from the closest point (not the start)
  // stops us from chasing a point we've already passed.
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    double d = std::hypot(p.x - x, p.y - y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }
  size_t target = path.poses.size() - 1;  // default: aim at the end of the path
  for (size_t i = closest; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    if (std::hypot(p.x - x, p.y - y) >= params_.lookahead_distance) {
      target = i;
      break;
    }
  }
  const auto& tp = path.poses[target].pose.position;

  // Target in the robot frame (x forward, y left), relative to the odometry point
  double dx = tp.x - x;
  double dy = tp.y - y;
  double local_x =  std::cos(yaw) * dx + std::sin(yaw) * dy;
  double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  double dist = std::hypot(local_x, local_y);
  double bearing = std::atan2(local_y, local_x);
  if (dist < 1e-6) return std::nullopt;

  geometry_msgs::msg::Twist cmd;

  // Odometry tracks the lidar, which sits axle_offset ahead of the middle of
  // the wheel axle that the robot turns about. Turning on the spot swings the
  // lidar around a circle of that radius, so it only helps if the goal is
  // outside that circle; a goal inside it stays behind the lidar however far
  // we turn, and we have to back up to it instead (handled below).
  double axle_x = x - params_.axle_offset * std::cos(yaw);
  double axle_y = y - params_.axle_offset * std::sin(yaw);
  bool goal_outside_turn_circle = std::hypot(end.x - axle_x, end.y - axle_y) > params_.axle_offset;
  if (std::abs(bearing) > params_.rotate_in_place_angle && goal_outside_turn_circle) {
    cmd.angular.z = std::copysign(params_.max_angular_speed, bearing);
    return cmd;
  }

  // Slow down near the end of the path so we don't overshoot the goal
  double speed = params_.linear_speed * std::clamp(dist_to_end / params_.slowdown_distance, 0.3, 1.0);

  // Driving at v while turning at w moves the lidar forward at v and sideways
  // at w * axle_offset. Pick v and w so the lidar heads straight for the
  // lookahead point at `speed`. (Classic pure pursuit fits an arc for the
  // axle instead, which would put the axle, not the lidar, on the goal.)
  // v comes out negative only when the target is behind us, i.e. backing up
  // to a goal inside the turning circle.
  double v = speed * local_x / dist;
  double w = speed * local_y / (dist * params_.axle_offset);

  // Turning too fast: slow both down together so the lidar keeps its heading
  if (std::abs(w) > params_.max_angular_speed) {
    double scale = params_.max_angular_speed / std::abs(w);
    v *= scale;
    w *= scale;
  }

  cmd.linear.x = v;
  cmd.angular.z = w;
  return cmd;
}

}
