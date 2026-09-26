#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    struct Params {
      double lookahead_distance = 1.0;     // m ahead of the middle of the wheel axle to aim at
      double linear_speed = 1.0;           // m/s cruise speed
      double max_angular_speed = 1.2;      // rad/s; sharper turns slow the robot down instead
      double max_acceleration = 1.0;       // m/s^2 when speeding up
      double max_deceleration = 2.0;       // m/s^2 when slowing down
      double max_angular_acceleration = 3.0;  // rad/s^2
      double goal_tolerance = 0.2;         // m, stop when this close to the end of the path
      double slowdown_distance = 1.0;      // m, start slowing down this far from the end or a corner
      double rotate_in_place_angle = 0.8;  // rad, turn on the spot if the target is further off than this
      double corner_angle = 0.35;          // rad, a heading jump this big in the path is a corner to turn on the spot at
      double turn_margin = 0.2;            // m the body must stay clear of obstacles while turning on the spot
      double axle_offset = 1.3;            // m, how far the odometry point (lidar) is ahead of the wheel axle
    };
    void setParams(const Params& params) { params_ = params; }

    // Obstacles to check before turning on the spot: every map cell with cost 100
    void setMap(const nav_msgs::msg::OccupancyGrid& map);

    // Pure pursuit for the middle of the wheel axle, the point the robot turns
    // about. x, y, yaw is the odometry (lidar) pose in the path's frame; dt is
    // the time since the last command. Returns nullopt once the end of the
    // path is reached.
    std::optional<geometry_msgs::msg::Twist> computeCommand(const nav_msgs::msg::Path& path,
                                                            double x, double y, double yaw, double dt);

    // Whether the body stays turn_margin clear of every obstacle while
    // turning on the spot by `turn` (rad, + is left), or backing up `distance`
    // m. An obstacle already closer than that may not get any closer.
    bool turnClear(double axle_x, double axle_y, double yaw, double turn) const;
    bool reverseClear(double axle_x, double axle_y, double yaw, double distance) const;

  private:
    rclcpp::Logger logger_;
    Params params_;
    std::vector<std::pair<double, double>> obstacles_;  // world positions of the map's obstacle cells
    double last_speed_ = 0.0;
    double last_turn_rate_ = 0.0;
    int spin_dir_ = 0;  // +1 turning on the spot to the left, -1 to the right, 0 not
    rclcpp::Clock clock_;  // for throttled logging

    geometry_msgs::msg::Twist turnInPlace(double axle_x, double axle_y, double yaw, double turn, double dt);
    // The command to send, after limiting how fast speed and turn rate change
    geometry_msgs::msg::Twist ramped(double v, double w, double dt);
    // Obstacles within reach of the body from this axle position, in the world frame
    std::vector<std::pair<double, double>> nearbyObstacles(double axle_x, double axle_y) const;
};

// Distance (m) from a point in the robot's frame (x forward, y left, origin at
// the middle of the wheel axle) to its body: the 2.0 x 1.0 m chassis from
// 0.5 m behind the axle to 1.5 m ahead, and the 0.8 m wheels 0.2 m either
// side of it (robot_env.sdf). 0 if the point is inside.
double distanceToBody(double bx, double by);

}

#endif
