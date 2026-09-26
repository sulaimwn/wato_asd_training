#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

namespace
{

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double yawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// The body's parts as (x0, x1, y0, y1) rectangles in the axle frame
const double kParts[3][4] = {
  {-0.5, 1.5, -0.5, 0.5},   // chassis
  {-0.4, 0.4, 0.5, 0.7},    // left wheel
  {-0.4, 0.4, -0.7, -0.5},  // right wheel
};
constexpr double kBodyReach = 1.6;  // m, farthest point of the body from the axle (a front corner, 1.58 m)

}  // namespace

double distanceToBody(double bx, double by) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& p : kParts) {
    const double dx = std::max({p[0] - bx, 0.0, bx - p[1]});
    const double dy = std::max({p[2] - by, 0.0, by - p[3]});
    best = std::min(best, std::hypot(dx, dy));
  }
  return best;
}

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
  obstacles_.clear();
  const double res = map.info.resolution;
  for (size_t i = 0; i < map.data.size(); ++i) {
    if (map.data[i] >= 100) {
      obstacles_.emplace_back(map.info.origin.position.x + (i % map.info.width + 0.5) * res,
                              map.info.origin.position.y + (i / map.info.width + 0.5) * res);
    }
  }
}

std::vector<std::pair<double, double>> ControlCore::nearbyObstacles(double axle_x, double axle_y) const {
  const double reach = kBodyReach + params_.turn_margin + 0.5;
  std::vector<std::pair<double, double>> near;
  for (const auto& o : obstacles_) {
    if (std::abs(o.first - axle_x) < reach && std::abs(o.second - axle_y) < reach) near.push_back(o);
  }
  return near;
}

bool ControlCore::turnClear(double axle_x, double axle_y, double yaw, double turn) const {
  const auto near = nearbyObstacles(axle_x, axle_y);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(turn) / (M_PI / 60))));  // every 3 degrees
  for (const auto& o : near) {
    const double dx = o.first - axle_x;
    const double dy = o.second - axle_y;
    if (std::hypot(dx, dy) > kBodyReach + params_.turn_margin + 0.1) continue;
    double allowed = -1.0;  // closest this obstacle may get
    for (int s = 0; s <= steps; ++s) {
      const double h = yaw + turn * s / steps;
      const double d = distanceToBody(std::cos(h) * dx + std::sin(h) * dy, -std::sin(h) * dx + std::cos(h) * dy);
      if (s == 0) {
        allowed = std::min(params_.turn_margin, d - 0.02);
      } else if (d < allowed) {
        return false;
      }
    }
  }
  return true;
}

bool ControlCore::reverseClear(double axle_x, double axle_y, double yaw, double distance) const {
  const auto near = nearbyObstacles(axle_x, axle_y);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / 0.05)));
  for (const auto& o : near) {
    double allowed = -1.0;
    for (int k = 0; k <= steps; ++k) {
      // The obstacle as seen from the robot backed up by k steps
      const double dx = o.first - (axle_x - c * distance * k / steps);
      const double dy = o.second - (axle_y - s * distance * k / steps);
      const double d = distanceToBody(c * dx + s * dy, -s * dx + c * dy);
      if (k == 0) {
        allowed = std::min(params_.turn_margin, d - 0.02);
      } else if (d < allowed) {
        return false;
      }
    }
  }
  return true;
}

geometry_msgs::msg::Twist ControlCore::ramped(double v, double w, double dt) {
  // Speed and turn rate change gradually. Besides being kinder to the robot,
  // the map depends on it: it places each scan using odometry interpolated
  // between readings 0.1 s apart, which is only right if the turn rate didn't
  // jump in between. Starting or stopping a spin instantly put walls 15 m
  // away up to a metre out of place.
  v = std::clamp(v, last_speed_ - params_.max_deceleration * dt, last_speed_ + params_.max_acceleration * dt);
  w = std::clamp(w, last_turn_rate_ - params_.max_angular_acceleration * dt,
                 last_turn_rate_ + params_.max_angular_acceleration * dt);
  last_speed_ = v;
  last_turn_rate_ = w;
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = w;
  return cmd;
}

geometry_msgs::msg::Twist ControlCore::turnInPlace(double axle_x, double axle_y, double yaw, double turn, double dt) {
  // Come to a stop first
  if (std::abs(last_speed_) > 0.02) return ramped(0.0, 0.0, dt);
  // Once turning one way, keep going that way (short of small corrections for
  // overshooting). Where the short way round only just has room, choosing
  // afresh every cycle flipped between the two and the robot just rocked.
  if (spin_dir_ != 0 && (turn > 0) != (spin_dir_ > 0) && std::abs(turn) > 0.5) {
    turn += spin_dir_ * 2.0 * M_PI;
  }
  // Turning on the spot swings the front corners round a 1.6 m circle. Go the
  // short way round if the body has room, else the long way, else back up a bit.
  if (!turnClear(axle_x, axle_y, yaw, turn)) {
    const double other_way = turn - std::copysign(2.0 * M_PI, turn);
    if (turnClear(axle_x, axle_y, yaw, other_way)) {
      turn = other_way;
    } else if (reverseClear(axle_x, axle_y, yaw, 0.3)) {
      RCLCPP_INFO_THROTTLE(logger_, clock_, 2000, "No room to turn here, backing up");
      spin_dir_ = 0;
      return ramped(-0.25, 0.0, dt);
    } else {
      RCLCPP_WARN_THROTTLE(logger_, clock_, 2000, "No room to turn or back up, waiting");
      spin_dir_ = 0;
      return ramped(0.0, 0.0, dt);
    }
  }
  spin_dir_ = turn > 0 ? 1 : -1;
  // Ease off towards the end of the turn so it doesn't overshoot
  return ramped(0.0, std::copysign(std::clamp(1.5 * std::abs(turn), 0.3, params_.max_angular_speed), turn), dt);
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeCommand(const nav_msgs::msg::Path& path,
                                                                     double x, double y, double yaw, double dt) {
  if (path.poses.empty()) {
    last_speed_ = last_turn_rate_ = 0.0;
    spin_dir_ = 0;
    return std::nullopt;
  }

  // Everything is measured from the middle of the wheel axle. Odometry is the
  // lidar, axle_offset ahead of it.
  const double ax = x - params_.axle_offset * std::cos(yaw);
  const double ay = y - params_.axle_offset * std::sin(yaw);
  const auto& end = path.poses.back().pose.position;
  const double dist_to_end = std::hypot(end.x - ax, end.y - ay);
  if (dist_to_end < params_.goal_tolerance) {
    last_speed_ = last_turn_rate_ = 0.0;
    spin_dir_ = 0;
    return std::nullopt;
  }

  const size_t n = path.poses.size();
  auto pos = [&](size_t i) { return path.poses[i].pose.position; };
  auto heading = [&](size_t i) { return yawOf(path.poses[i].pose.orientation); };

  // Closest path point to the robot. Searching on from there (not from the
  // start) stops us from chasing a point we've already passed.
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n; ++i) {
    const double d = std::hypot(pos(i).x - ax, pos(i).y - ay);
    if (d < best) {
      best = d;
      closest = i;
    }
  }

  // The next sharp corner, where the planner found no room for an arc and
  // wants the robot to stop and turn on the spot (the path's heading jumps
  // there). Skip it if we're standing on it already facing the new way.
  size_t corner = n;
  for (size_t i = closest; i + 1 < n; ++i) {
    if (std::abs(wrap(heading(i + 1) - heading(i))) < params_.corner_angle) continue;
    const bool at_it = std::hypot(pos(i).x - ax, pos(i).y - ay) < 0.3;
    if (at_it && std::abs(wrap(heading(i + 1) - yaw)) < 0.1) continue;
    corner = i;
    break;
  }

  double speed = params_.linear_speed * std::clamp(dist_to_end / params_.slowdown_distance, 0.25, 1.0);
  double tx = 0.0;
  double ty = 0.0;
  bool have_target = false;
  if (corner < n) {
    const double hin = heading(corner);
    const double cx = pos(corner).x - ax;
    const double cy = pos(corner).y - ay;
    const double ahead = std::cos(hin) * cx + std::sin(hin) * cy;  // how far the corner still is along the line in
    if (ahead < 0.1) {
      return turnInPlace(ax, ay, yaw, wrap(heading(corner + 1) - yaw), dt);  // there: turn to the next line
    }
    if (std::hypot(cx, cy) < params_.lookahead_distance) {
      // Don't look round the corner (that's what cuts it): aim along the line
      // into it, past the corner, so we drive straight up to it
      tx = pos(corner).x + std::cos(hin) * (params_.lookahead_distance - ahead);
      ty = pos(corner).y + std::sin(hin) * (params_.lookahead_distance - ahead);
      have_target = true;
    }
    speed = std::min(speed, params_.linear_speed * std::clamp(ahead / params_.slowdown_distance, 0.25, 1.0));
  }
  if (!have_target) {
    // First point at least one lookahead away, or the end of the path
    size_t target = n - 1;
    for (size_t i = closest; i < n; ++i) {
      if (std::hypot(pos(i).x - ax, pos(i).y - ay) >= params_.lookahead_distance) {
        target = i;
        break;
      }
    }
    tx = pos(target).x;
    ty = pos(target).y;
  }

  // Target in the robot frame (x forward, y left)
  const double dx = tx - ax;
  const double dy = ty - ay;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double dist_sq = local_x * local_x + local_y * local_y;
  if (dist_sq < 1e-12) return std::nullopt;
  // Turn on the spot if the target is well off to the side, and once turning,
  // keep at it until facing it (so it doesn't flick between turning and driving)
  const double bearing = std::atan2(local_y, local_x);
  if (std::abs(bearing) > params_.rotate_in_place_angle || (spin_dir_ != 0 && std::abs(bearing) > 0.15)) {
    return turnInPlace(ax, ay, yaw, bearing, dt);
  }
  spin_dir_ = 0;

  // Pure pursuit: the arc from the axle through the target point
  const double curvature = 2.0 * local_y / dist_sq;
  double v = speed;
  if (std::abs(curvature) * v > params_.max_angular_speed) {
    v = params_.max_angular_speed / std::abs(curvature);  // tight bend: slow down, keep the arc
  }
  // Keep to the arc while the turn rate catches up: if it can't change fast
  // enough, drive slower instead of swinging wide
  v = std::clamp(v, last_speed_ - params_.max_deceleration * dt, last_speed_ + params_.max_acceleration * dt);
  const double w_reach = std::abs(last_turn_rate_) + params_.max_angular_acceleration * dt;
  if (std::abs(curvature * v) > w_reach) v = std::max(0.0, w_reach / std::abs(curvature));
  return ramped(v, curvature * v, dt);
}

}
