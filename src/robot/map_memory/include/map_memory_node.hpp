#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

  private:
    // Robot pose in the world frame at a given time (seconds)
    struct Pose2D {
      double t;
      double x;
      double y;
      double yaw;
    };

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Recent costmaps, newest last, waiting for the odometry to catch up with them
    std::deque<nav_msgs::msg::OccupancyGrid> recent_costmaps_;

    // Recent odometry, used to look up the pose at a costmap's timestamp
    std::deque<Pose2D> odom_history_;

    // Robot position and time at the last map update
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    rclcpp::Time last_update_time_;
    bool first_update_done_ = false;

    double distance_threshold_;
    double max_update_interval_;

    bool poseAt(double t, Pose2D& pose) const;
    void publishMap();
};

#endif
