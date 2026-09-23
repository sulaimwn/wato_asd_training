#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    struct Params {
      double lookahead_distance = 1.0;   // m, measured from the odometry (lidar) point
      double linear_speed = 0.5;         // m/s cruise speed
      double max_angular_speed = 1.0;    // rad/s
      double goal_tolerance = 0.2;       // m, stop when this close to the end of the path
      double slowdown_distance = 1.5;    // m, start slowing down this far from the end
      double rotate_in_place_angle = 1.05;  // rad, turn on the spot if the target is further off than this
      double axle_offset = 1.3;          // m, how far the odometry point is ahead of the wheel axle
    };
    void setParams(const Params& params) { params_ = params; }

    // Pure pursuit: pick a lookahead point on the path and steer the odometry
    // point (the lidar) towards it. x, y, yaw is the odometry pose in the
    // path's frame. Returns nullopt once the end of the path is reached.
    std::optional<geometry_msgs::msg::Twist> computeCommand(const nav_msgs::msg::Path& path,
                                                            double x, double y, double yaw);

  private:
    rclcpp::Logger logger_;
    Params params_;
};

}

#endif
