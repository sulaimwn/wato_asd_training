#include "costmap_core.hpp"

#include <algorithm>
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

// Mark every cell a beam passed through on its way out as free (0). A cell
// counts only if the beams on BOTH sides of its direction reach past it: with
// just the nearest beam, cells on a wall seen at a shallow angle (where hits
// land far apart) would be marked free between the hits.
void CostmapCore::markFreeSpace(std::vector<int8_t>& grid, const sensor_msgs::msg::LaserScan& scan) {
  const int n = static_cast<int>(scan.ranges.size());
  if (n < 2 || scan.angle_increment <= 0.0) return;
  const bool full_circle =
      scan.angle_max - scan.angle_min + scan.angle_increment >= 2.0 * M_PI - 1e-3;

  // How far out each beam proves the space is empty
  std::vector<double> reach(n, 0.0);
  for (int i = 0; i < n; ++i) {
    double r = scan.ranges[i];
    if (std::isnan(r) || r < scan.range_min) continue;  // unusable reading: proves nothing
    reach[i] = std::min(r, static_cast<double>(scan.range_max));  // inf = nothing within range
  }

  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      // Cell center relative to the sensor
      double x = origin_x_ + (gx + 0.5) * resolution_;
      double y = origin_y_ + (gy + 0.5) * resolution_;
      double r = std::hypot(x, y);

      // Which two beams this direction falls between
      double beam = (std::atan2(y, x) - scan.angle_min) / scan.angle_increment;
      int lo = static_cast<int>(std::floor(beam));
      int hi = lo + 1;
      if (full_circle) {
        lo = ((lo % n) + n) % n;
        hi = ((hi % n) + n) % n;
      } else if (lo < 0 || hi >= n) {
        continue;  // outside the scan's field of view
      }

      // Stop one cell short of the hit so the hit's own cell is never free
      if (r < std::min(reach[lo], reach[hi]) - resolution_) grid[gy * width_ + gx] = 0;
    }
  }
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
        // Unknown cells (-1) get inflated too, so the far side of a wall we've
        // only seen from the front still keeps the robot at a distance
        if (cost > 0 && cost > grid[idx]) grid[idx] = static_cast<int8_t>(cost);
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  std::vector<int8_t> grid(width_ * height_, -1);  // -1 = unknown until a beam sees it

  markFreeSpace(grid, *scan);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) continue;
    double angle = scan->angle_min + i * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    int gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    int gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
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
