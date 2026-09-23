#include "explorer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

ExplorerCore::ExplorerCore(const rclcpp::Logger& logger) : logger_(logger) {}

std::vector<Frontier> ExplorerCore::findFrontiers(const nav_msgs::msg::OccupancyGrid& map,
                                                  double robot_x, double robot_y,
                                                  std::vector<std::pair<double, double>>* cells) const {
  std::vector<Frontier> frontiers;
  if (cells) cells->clear();

  const int width = map.info.width;
  const int height = map.info.height;
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  if (width == 0 || height == 0 || res <= 0.0 ||
      map.data.size() != static_cast<size_t>(width) * height) {
    return frontiers;
  }

  const int rx = static_cast<int>(std::floor((robot_x - ox) / res));
  const int ry = static_cast<int>(std::floor((robot_y - oy) / res));
  if (rx < 0 || rx >= width || ry < 0 || ry >= height) return frontiers;

  auto cost = [&](int i) { return static_cast<int>(map.data[i]); };
  auto isFree = [&](int i) { return cost(i) >= 0 && cost(i) < params_.free_threshold; };
  const double escape_sq = params_.escape_radius * params_.escape_radius;
  auto passable = [&](int x, int y) {
    int i = y * width + x;
    if (isFree(i)) return true;
    if (cost(i) < 0 || cost(i) >= 100) return false;  // unknown, or an obstacle
    double dx = (x - rx) * res, dy = (y - ry) * res;
    return dx * dx + dy * dy <= escape_sq;
  };

  // 1. Flood fill out from the robot through passable cells. steps[i] ends up
  //    as the number of moves to reach cell i, or -1 if it can't be reached.
  std::vector<int> steps(width * height, -1);
  std::queue<int> queue;
  steps[ry * width + rx] = 0;
  queue.push(ry * width + rx);
  while (!queue.empty()) {
    int i = queue.front();
    queue.pop();
    int x = i % width, y = i / width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        int nx = x + dx, ny = y + dy;
        if ((dx == 0 && dy == 0) || nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        int ni = ny * width + nx;
        if (steps[ni] != -1 || !passable(nx, ny)) continue;
        steps[ni] = steps[i] + 1;
        queue.push(ni);
      }
    }
  }

  // 2. Frontier cells: reachable, (really) free cells with an unknown neighbour
  std::vector<bool> is_frontier(width * height, false);
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int i = y * width + x;
      if (steps[i] < 0 || cost(i) < 0 || cost(i) > params_.frontier_max_cost) continue;
      if (cost(i - 1) == -1 || cost(i + 1) == -1 || cost(i - width) == -1 || cost(i + width) == -1) {
        is_frontier[i] = true;
      }
    }
  }

  // 3. Group touching frontier cells into patches, and aim each patch's goal
  //    at its member cell closest to the patch's average position
  std::vector<bool> grouped(width * height, false);
  for (int start = 0; start < width * height; ++start) {
    if (!is_frontier[start] || grouped[start]) continue;

    std::vector<int> patch;
    grouped[start] = true;
    queue.push(start);
    while (!queue.empty()) {
      int i = queue.front();
      queue.pop();
      patch.push_back(i);
      int x = i % width, y = i / width;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          int nx = x + dx, ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          int ni = ny * width + nx;
          if (!is_frontier[ni] || grouped[ni]) continue;
          grouped[ni] = true;
          queue.push(ni);
        }
      }
    }
    if (static_cast<int>(patch.size()) < params_.min_frontier_size) continue;

    double mean_x = 0.0, mean_y = 0.0;
    for (int i : patch) {
      mean_x += i % width;
      mean_y += i / width;
    }
    mean_x /= patch.size();
    mean_y /= patch.size();

    int goal = patch.front();
    double best = std::numeric_limits<double>::infinity();
    for (int i : patch) {
      double d = std::hypot(i % width - mean_x, i / width - mean_y);
      if (d < best) {
        best = d;
        goal = i;
      }
    }

    Frontier f;
    f.goal_x = ox + (goal % width + 0.5) * res;
    f.goal_y = oy + (goal / width + 0.5) * res;
    f.distance = steps[goal] * res;
    f.size = static_cast<int>(patch.size());
    frontiers.push_back(f);

    if (cells) {
      for (int i : patch) cells->emplace_back(ox + (i % width + 0.5) * res, oy + (i / width + 0.5) * res);
    }
  }

  std::sort(frontiers.begin(), frontiers.end(),
            [](const Frontier& a, const Frontier& b) { return a.distance < b.distance; });
  return frontiers;
}

bool ExplorerCore::nearAny(const std::vector<std::pair<double, double>>& points,
                           double x, double y, double radius) {
  const double r_sq = radius * radius;
  for (const auto& p : points) {
    double dx = p.first - x, dy = p.second - y;
    if (dx * dx + dy * dy <= r_sq) return true;
  }
  return false;
}

}
