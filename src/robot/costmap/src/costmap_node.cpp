#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // 0.1 m/cell, 30m x 30m grid, 1.6 m inflation. The planner blocks cost >= 50,
  // i.e. within 0.8 m of an obstacle: the wheels stick out 0.7 m either side
  // of the lidar's track, so this leaves ~0.1 m spare before the planner's
  // extra cost for being near obstacles kicks in.
  costmap_.initCostmap(0.1, 300, 300, 1.6);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid costmap_msg = costmap_.processScan(scan);
  // Keep the scan's (sim) time so map_memory can match it to the odometry at that instant
  costmap_msg.header.stamp = scan->header.stamp;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
