#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Set grid size/resolution and how far to inflate around obstacles
    void initCostmap(double resolution, int width, int height, double inflation_radius);

    // Turn one laser scan into an OccupancyGrid: -1 = never seen by a beam,
    // 0 = a beam passed through it (free), 1..100 = obstacle / inflation cost
    nav_msgs::msg::OccupancyGrid processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    rclcpp::Logger logger_;

    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double inflation_radius_;
    int max_cost_;

    void markFreeSpace(std::vector<int8_t>& grid, const sensor_msgs::msg::LaserScan& scan);
    void markObstacle(std::vector<int8_t>& grid, int gx, int gy);
    void inflate(std::vector<int8_t>& grid);
};

}

#endif
