#ifndef EXPLORER_NODE_HPP_
#define EXPLORER_NODE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "explorer_core.hpp"

// Frontier-based exploration: repeatedly sends the planner to the nearest
// boundary between explored and unexplored space until there is none left,
// then drives back to where it started. Off until something publishes true on
// /explore/enable; false (or any goal it didn't send itself) turns it off.
class ExplorerNode : public rclcpp::Node {
  public:
    ExplorerNode();

  private:
    enum class State { IDLE, EXPLORING, RETURNING_HOME, COMPLETE };

    robot::ExplorerCore explorer_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::IDLE;

    nav_msgs::msg::OccupancyGrid map_;
    bool have_map_ = false;
    double robot_x_ = 0.0, robot_y_ = 0.0;
    bool have_odom_ = false;

    // Where exploration started, to return to at the end
    double home_x_ = 0.0, home_y_ = 0.0;
    bool have_home_ = false;

    // The goal we are currently driving to
    bool have_goal_ = false;
    double goal_x_ = 0.0, goal_y_ = 0.0;
    double path_length_ = -1.0;          // m, length of the planner's latest route to it (-1: none yet)
    double best_remaining_ = 0.0;        // shortest distance-to-go seen so far
    rclcpp::Time last_progress_time_;     // when best_remaining_ last improved
    rclcpp::Time last_path_time_;         // when the planner last sent a non-empty path

    // Goals we've reached or given up on; nearby frontiers are skipped
    std::vector<std::pair<double, double>> blacklist_;
    int rounds_without_frontier_ = 0;

    // Parameters
    double goal_reached_dist_;
    double frontier_resolved_radius_;
    double blacklist_radius_;
    double stuck_timeout_;
    double path_fail_timeout_;
    bool return_home_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void tick();

    void start();
    void setState(State state, const std::string& why);
    void sendGoal(double x, double y);
    void cancelGoal();
    bool pickNextFrontier(const std::vector<robot::Frontier>& frontiers);
    bool blacklisted(double x, double y) const;
    void publishMarkers(const std::vector<std::pair<double, double>>& cells);
    static const char* stateName(State state);
};

#endif
