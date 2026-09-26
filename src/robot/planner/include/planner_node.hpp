#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void cancelCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_ = State::WAITING_FOR_GOAL;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_frame_ = "sim_world";
    bool have_map_ = false;

    geometry_msgs::msg::PointStamped goal_;
    double target_x_ = 0.0;  // where the latest path ends: within arrive_radius of the
    double target_y_ = 0.0;  //   goal, or the nearest spot to it the robot can reach
    rclcpp::Time goal_start_time_;

    // The middle of the wheel axle, the point the robot turns about.
    // Odometry reports the lidar, axle_offset_ ahead of it.
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    bool have_odom_ = false;

    double goal_tolerance_;
    double goal_timeout_;
    double axle_offset_;
    double arrive_radius_;
    double body_margin_;
    double switch_distance_;

    nav_msgs::msg::Path current_path_;  // the route we're following, as last published

    bool goalReached() const;
    void planPath();
    bool keepCurrentPath(const nav_msgs::msg::Path& fresh);
    void publishEmptyPath();  // tells the controller to stop
};

#endif
