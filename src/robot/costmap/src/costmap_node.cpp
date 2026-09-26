#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // 0.1 m/cell, 30m x 30m grid, 1.6 m inflation. The planner works out for
  // itself where the robot's body fits from the lidar hits (cost 100); the
  // cost fading out around them only makes it prefer routes with room to spare.
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
