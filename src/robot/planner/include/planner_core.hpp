#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex &other) const { return !(*this == other); }
};

// Node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
  double h_score;  // heuristic part, used to break ties

  AStarNode(CellIndex idx, double f, double h) : index(idx), f_score(f), h_score(h) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) const
  {
    // In open space many staircase paths have exactly the same f. Preferring
    // the node closer to the goal picks the same one every time, so the path
    // doesn't flip between replans.
    if (std::abs(a.f_score - b.f_score) > 1e-9) return a.f_score > b.f_score;
    return a.h_score > b.h_score;
  }
};

class PlannerCore {
  public:
    enum class Result { OK, NO_PATH, START_INVALID, GOAL_INVALID };

    explicit PlannerCore(const rclcpp::Logger& logger);

    // obstacle_threshold: cells at or above this cost are blocked
    // cost_weight: how strongly to avoid (unblocked) high-cost cells
    // escape_radius: if the start (or goal) is itself blocked, blocked but
    //   non-lethal cells this close (m) to it are allowed as long as they cost
    //   no more than it does, so the robot can plan out of an inflated zone
    void setParams(int obstacle_threshold, double cost_weight, double escape_radius);

    // Plan on the map with A* from start to goal (world coordinates, map frame).
    // GOAL_INVALID: goal is off the map or inside an obstacle.
    // START_INVALID: robot is off the map.
    Result planPath(const nav_msgs::msg::OccupancyGrid& map,
                    double start_x, double start_y,
                    double goal_x, double goal_y,
                    nav_msgs::msg::Path& path);

  private:
    rclcpp::Logger logger_;

    int obstacle_threshold_ = 50;
    double cost_weight_ = 3.0;
    double escape_radius_ = 1.0;

    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& cell) const;
};

}

#endif
