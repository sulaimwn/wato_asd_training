#include "costmap_core.hpp"

#include <cmath>
#include <vector>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
  initCostmap(0.1, 300, 300, 1.0);  // defaults
}

void CostmapCore::initCostmap(double resolution, int width, int height, double inflation_radius) {
  resolution_ = resolution;
  width_  = width;
  height_ = height;
  // Put the robot (sensor) at the center of the grid
  origin_x_ = -(width_  * resolution_) / 2.0;
  origin_y_ = -(height_ * resolution_) / 2.0;
  inflation_radius_ = inflation_radius;
  max_cost_ = 100;
}

void CostmapCore::markObstacle(std::vector<int8_t>& grid, int gx, int gy) {
  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return;
  grid[gy * width_ + gx] = max_cost_;
}

void CostmapCore::inflate(std::vector<int8_t>& grid) {
  int cell_radius = static_cast<int>(inflation_radius_ / resolution_);

  // Collect obstacle cells first so we don't inflate off already-inflated cells
  std::vector<std::pair<int,int>> obstacles;
  for (int y = 0; y < height_; ++y)
    for (int x = 0; x < width_; ++x)
      if (grid[y * width_ + x] == max_cost_)
        obstacles.emplace_back(x, y);

  for (auto& o : obstacles) {
    int ox = o.first, oy = o.second;
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
        int nx = ox + dx, ny = oy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
        double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
        if (dist > inflation_radius_) continue;
        int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
        int idx = ny * width_ + nx;
        if (cost > grid[idx]) grid[idx] = static_cast<int8_t>(cost);
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  std::vector<int8_t> grid(width_ * height_, 0);  // 0 = free

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max) continue;
    double angle = scan->angle_min + i * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    int gx = static_cast<int>((x - origin_x_) / resolution_);
    int gy = static_cast<int>((y - origin_y_) / resolution_);
    markObstacle(grid, gx, gy);
  }

  inflate(grid);

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = scan->header.frame_id;
  msg.info.resolution = resolution_;
  msg.info.width  = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid;
  return msg;
}

}
