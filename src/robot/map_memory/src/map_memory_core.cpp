#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initMap(double resolution, int width, int height,
                            double origin_x, double origin_y, const std::string& frame_id) {
  global_map_.header.frame_id = frame_id;
  global_map_.info.resolution = resolution;
  global_map_.info.width  = width;
  global_map_.info.height = height;
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width * height, -1);  // -1 = unknown
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  const double c_res = costmap.info.resolution;
  const int c_w = costmap.info.width;
  const int c_h = costmap.info.height;
  const double c_ox = costmap.info.origin.position.x;
  const double c_oy = costmap.info.origin.position.y;
  if (c_res <= 0.0 || c_w == 0 || c_h == 0 ||
      costmap.data.size() != static_cast<size_t>(c_w) * c_h) {
    RCLCPP_WARN(logger_, "Ignoring malformed costmap");
    return;
  }

  const double m_res = global_map_.info.resolution;
  const int m_w = global_map_.info.width;
  const int m_h = global_map_.info.height;
  const double m_ox = global_map_.info.origin.position.x;
  const double m_oy = global_map_.info.origin.position.y;

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  // Only visit map cells that could lie under the (rotated) costmap: take the
  // farthest costmap corner from the robot as the radius of a bounding box
  double reach = 0.0;
  for (double cx : {c_ox, c_ox + c_w * c_res})
    for (double cy : {c_oy, c_oy + c_h * c_res})
      reach = std::max(reach, std::hypot(cx, cy));

  const int min_mx = std::max(0,       static_cast<int>(std::floor((robot_x - reach - m_ox) / m_res)));
  const int max_mx = std::min(m_w - 1, static_cast<int>(std::floor((robot_x + reach - m_ox) / m_res)));
  const int min_my = std::max(0,       static_cast<int>(std::floor((robot_y - reach - m_oy) / m_res)));
  const int max_my = std::min(m_h - 1, static_cast<int>(std::floor((robot_y + reach - m_oy) / m_res)));

  // Pass 1: for each map cell, look up which costmap cell covers its center.
  // Going map -> costmap means rotation never leaves holes in the map.
  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      // Map cell center in the world frame, relative to the robot
      double dx = m_ox + (mx + 0.5) * m_res - robot_x;
      double dy = m_oy + (my + 0.5) * m_res - robot_y;

      // Rotate into the robot (costmap) frame
      double local_x =  cos_yaw * dx + sin_yaw * dy;
      double local_y = -sin_yaw * dx + cos_yaw * dy;

      int cx = static_cast<int>(std::floor((local_x - c_ox) / c_res));
      int cy = static_cast<int>(std::floor((local_y - c_oy) / c_res));
      if (cx < 0 || cx >= c_w || cy < 0 || cy >= c_h) continue;

      int8_t cost = costmap.data[cy * c_w + cx];
      if (cost < 0) continue;  // unknown in the costmap: keep what we had

      // Keep the highest cost seen. The world is static, and a beam grazing
      // past an obstacle can mark part of its cell free from one angle, so a
      // free reading is never allowed to erase an obstacle we saw earlier.
      int8_t& cell = global_map_.data[my * m_w + mx];
      cell = std::max(cell, cost);
    }
  }

  // Pass 2: with both grids at the same resolution but rotated, some costmap
  // cells contain no map cell center and get skipped by pass 1. Push every
  // non-free costmap cell forward into the map so no lidar hit is dropped.
  for (int cy = 0; cy < c_h; ++cy) {
    for (int cx = 0; cx < c_w; ++cx) {
      int8_t cost = costmap.data[cy * c_w + cx];
      if (cost <= 0) continue;

      // Costmap cell center in the robot frame, rotated into the world frame
      double local_x = c_ox + (cx + 0.5) * c_res;
      double local_y = c_oy + (cy + 0.5) * c_res;
      double wx = robot_x + cos_yaw * local_x - sin_yaw * local_y;
      double wy = robot_y + sin_yaw * local_x + cos_yaw * local_y;

      int mx = static_cast<int>(std::floor((wx - m_ox) / m_res));
      int my = static_cast<int>(std::floor((wy - m_oy) / m_res));
      if (mx < 0 || mx >= m_w || my < 0 || my >= m_h) continue;

      int8_t& cell = global_map_.data[my * m_w + mx];
      cell = std::max(cell, cost);
    }
  }
}

}
