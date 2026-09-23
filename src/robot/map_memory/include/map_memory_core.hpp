#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Set up an empty (all unknown) global map in the world frame
    void initMap(double resolution, int width, int height,
                 double origin_x, double origin_y, const std::string& frame_id);

    // Merge a robot-centered costmap into the global map, given the robot's
    // pose (x, y, yaw) in the world frame at the time of the costmap
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& getMap() const { return global_map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}

#endif
