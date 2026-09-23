#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::setParams(int obstacle_threshold, double cost_weight, double escape_radius) {
  obstacle_threshold_ = obstacle_threshold;
  cost_weight_ = cost_weight;
  escape_radius_ = escape_radius;
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& cell) const {
  const double res = map.info.resolution;
  cell.x = static_cast<int>(std::floor((wx - map.info.origin.position.x) / res));
  cell.y = static_cast<int>(std::floor((wy - map.info.origin.position.y) / res));
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

PlannerCore::Result PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                                          double start_x, double start_y,
                                          double goal_x, double goal_y,
                                          nav_msgs::msg::Path& path) {
  path.header.frame_id = map.header.frame_id;
  path.poses.clear();

  const int width = map.info.width;
  const int height = map.info.height;
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  if (width == 0 || height == 0 || res <= 0.0 ||
      map.data.size() != static_cast<size_t>(width) * height) {
    RCLCPP_WARN(logger_, "Map is empty or malformed");
    return Result::NO_PATH;
  }

  CellIndex start, goal;
  if (!worldToGrid(map, start_x, start_y, start)) return Result::START_INVALID;
  if (!worldToGrid(map, goal_x, goal_y, goal)) return Result::GOAL_INVALID;

  auto index = [width](int x, int y) { return y * width + x; };
  auto cellCost = [&](int x, int y) { return static_cast<int>(map.data[index(x, y)]); };

  // Unknown (-1) is free: we may plan through space we haven't seen yet.
  // Blocked cells are off-limits, except when the robot (or the goal) already
  // sits in a blocked zone: then nearby cells that are no closer to an
  // obstacle than it is are allowed, so it can get out (or in) without ever
  // moving towards what it's next to.
  const int start_cost = cellCost(start.x, start.y);
  const int goal_cost = cellCost(goal.x, goal.y);
  const double escape_sq = escape_radius_ * escape_radius_;
  auto traversable = [&](int x, int y) {
    int cost = cellCost(x, y);
    if (cost < obstacle_threshold_) return true;
    if (cost >= 100) return false;
    double wx = ox + (x + 0.5) * res;
    double wy = oy + (y + 0.5) * res;
    double ds = (wx - start_x) * (wx - start_x) + (wy - start_y) * (wy - start_y);
    double dg = (wx - goal_x) * (wx - goal_x) + (wy - goal_y) * (wy - goal_y);
    if (start_cost >= obstacle_threshold_ && cost <= start_cost && ds <= escape_sq) return true;
    if (goal_cost >= obstacle_threshold_ && cost <= goal_cost && dg <= escape_sq) return true;
    return false;
  };

  if (!traversable(goal.x, goal.y)) return Result::GOAL_INVALID;

  // Octile distance (m): exact cost of an 8-connected move on free cells, and
  // never an overestimate since every step costs at least its length
  auto heuristic = [&](int x, int y) {
    double dx = std::abs(x - goal.x);
    double dy = std::abs(y - goal.y);
    return res * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
  };

  const int n_cells = width * height;
  std::vector<double> g_score(n_cells, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(n_cells, -1);
  std::vector<bool> closed(n_cells, false);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  g_score[index(start.x, start.y)] = 0.0;
  open.emplace(start, heuristic(start.x, start.y), heuristic(start.x, start.y));

  bool found = false;
  while (!open.empty()) {
    CellIndex current = open.top().index;
    open.pop();

    int ci = index(current.x, current.y);
    if (closed[ci]) continue;  // stale entry, already expanded with a lower cost
    closed[ci] = true;

    if (current == goal) {
      found = true;
      break;
    }

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        int nx = current.x + dx;
        int ny = current.y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

        int ni = index(nx, ny);
        if (closed[ni] || !traversable(nx, ny)) continue;
        // Don't cut corners diagonally past a blocked cell
        if (dx != 0 && dy != 0 && (!traversable(current.x + dx, current.y) || !traversable(current.x, current.y + dy))) continue;

        // Step length (m), scaled up by the cost of the cell we step into
        double step = (dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0) * res;
        double penalty = 1.0 + cost_weight_ * std::max(0, cellCost(nx, ny)) / 100.0;
        double tentative_g = g_score[ci] + step * penalty;

        if (tentative_g < g_score[ni]) {
          g_score[ni] = tentative_g;
          came_from[ni] = ci;
          double h = heuristic(nx, ny);
          open.emplace(CellIndex(nx, ny), tentative_g + h, h);
        }
      }
    }
  }

  if (!found) return Result::NO_PATH;

  // Walk back from the goal to get the cells in order
  std::vector<int> cells;
  for (int i = index(goal.x, goal.y); i != -1; i = came_from[i]) cells.push_back(i);
  std::reverse(cells.begin(), cells.end());

  // Cell centers in the world frame, but starting exactly at the robot and
  // ending exactly on the requested goal
  std::vector<double> px, py, pcost;
  for (int c : cells) {
    px.push_back(ox + (c % width + 0.5) * res);
    py.push_back(oy + (c / width + 0.5) * res);
    pcost.push_back(std::max(0, static_cast<int>(map.data[c])));
  }
  px.front() = start_x;  py.front() = start_y;
  px.back() = goal_x;    py.back() = goal_y;

  // True if the straight line a -> b only crosses traversable cells that cost
  // no more than max_cost
  auto lineClear = [&](size_t a, size_t b, double max_cost) {
    double len = std::hypot(px[b] - px[a], py[b] - py[a]);
    int steps = std::max(1, static_cast<int>(std::ceil(len / (res * 0.25))));
    for (int s = 0; s <= steps; ++s) {
      double t = static_cast<double>(s) / steps;
      CellIndex c;
      if (!worldToGrid(map, px[a] + t * (px[b] - px[a]), py[a] + t * (py[b] - py[a]), c)) return false;
      if (!traversable(c.x, c.y) || std::max(0, cellCost(c.x, c.y)) > max_cost) return false;
    }
    return true;
  };

  // Line-of-sight shortcut: replace the grid staircase with straight segments.
  // A shortcut may not enter cells costlier than the part of the A* path it
  // replaces, so it never cuts closer to obstacles than A* chose to.
  std::vector<size_t> waypoints{0};
  size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    size_t next = anchor + 1;
    double seg_max = std::max(pcost[anchor], pcost[next]);
    for (size_t j = anchor + 2; j < cells.size(); ++j) {
      seg_max = std::max(seg_max, pcost[j]);
      if (!lineClear(anchor, j, seg_max)) break;
      next = j;
    }
    waypoints.push_back(next);
    anchor = next;
  }

  // Resample the segments every cell length so the controller gets a dense path
  auto addPose = [&](double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
  };
  double yaw = 0.0;
  for (size_t k = 0; k + 1 < waypoints.size(); ++k) {
    size_t a = waypoints[k], b = waypoints[k + 1];
    double len = std::hypot(px[b] - px[a], py[b] - py[a]);
    yaw = std::atan2(py[b] - py[a], px[b] - px[a]);
    int n = std::max(1, static_cast<int>(std::ceil(len / res)));
    for (int s = 0; s < n; ++s) {
      double t = static_cast<double>(s) / n;
      addPose(px[a] + t * (px[b] - px[a]), py[a] + t * (py[b] - py[a]), yaw);
    }
  }
  addPose(px.back(), py.back(), yaw);
  return Result::OK;
}

}
