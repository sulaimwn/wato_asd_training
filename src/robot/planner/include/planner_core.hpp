#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

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

// A circle in the robot's frame, centred on the middle of the wheel axle (the
// point it turns about): x forward, y left.
struct Circle
{
  double x;
  double y;
  double r;
};

// The robot's body as a set of circles that together cover it. From
// robot_env.sdf: a 2.0 x 1.0 m chassis from 0.5 m behind the axle to 1.5 m
// ahead of it, plus the wheels, 0.8 m across, sticking out 0.2 m either side.
// Circles make "is the body clear here?" one distance lookup per circle.
std::vector<Circle> bodyCircles();

class PlannerCore {
  public:
    enum class Result { OK, NO_PATH, START_INVALID, GOAL_INVALID };

    struct Params {
      double body_margin = 0.3;         // m to keep between the robot's body and any obstacle
      double cost_weight = 3.0;         // how strongly to prefer cells away from obstacles
      double escape_radius = 1.5;       // m; see planPath
      double arrive_radius = 0.25;      // m; see planPath
      double goal_search_radius = 1.5;  // m; see planPath
    };

    explicit PlannerCore(const rclcpp::Logger& logger);
    void setParams(const Params& params) { params_ = params; }

    // Takes a new map and works out every cell's distance to the nearest
    // obstacle (a lidar hit, cost 100). Planning uses the latest map given.
    void setMap(const nav_msgs::msg::OccupancyGrid& map);
    bool hasMap() const { return !dist_.empty(); }

    // Plans for the middle of the wheel axle, from the robot's pose to the
    // goal, keeping the whole body body_margin clear of obstacles.
    //
    // A* runs on the map's cells, and each step between cells is only allowed
    // if the body fits at both ends with the heading of that step, and any
    // sharp change of direction if the robot has room to turn on the spot
    // there. The grid path is then straightened along lines of sight, and
    // each corner is either rounded into an arc the body fits along, or left
    // sharp for the robot to stop and turn on the spot (the path's heading
    // jumps there) where it has room to.
    //
    // The path ends anywhere within arrive_radius of the goal the body fits,
    // or failing that, at the reachable spot nearest the goal if that's within
    // goal_search_radius. Where it ends is returned in goal_x/goal_y. If the
    // robot is already closer than body_margin to something, poses no tighter
    // than its current one are allowed within escape_radius of it, so it can
    // still get out.
    //
    // GOAL_INVALID: goal off the map, or nowhere near it reachable.
    // START_INVALID: robot off the map. NO_PATH: no map yet, or the robot is
    // hemmed in where it stands.
    Result planPath(double start_x, double start_y, double start_yaw,
                    double& goal_x, double& goal_y, nav_msgs::msg::Path& path);

    // Smallest gap (m) between the body at this pose and the nearest obstacle
    // on the map, measured from the circles (negative if they overlap it).
    double bodyClearance(double x, double y, double yaw) const;

    // Smallest bodyClearance while turning on the spot at (x, y) from heading
    // `from` to heading `to`, going whichever way round has more room
    double turnClearance(double x, double y, double from, double to) const;

    // Smallest bodyClearance along a path from pose `from` on, each pose
    // facing the way it points, including turning on the spot where the
    // path's heading jumps
    double pathClearance(const nav_msgs::msg::Path& path, size_t from) const;

  private:
    rclcpp::Logger logger_;
    Params params_;
    std::vector<Circle> circles_;
    double reach_ = 0.0;  // m from the axle to the farthest edge of any circle

    nav_msgs::msg::OccupancyGrid map_;
    int width_ = 0;
    int height_ = 0;
    double res_ = 0.1;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    std::vector<float> dist_;       // m from each cell's centre to the nearest obstacle cell's centre
    std::vector<float> clearance_;  // bodyClearance at each cell centre for each of the 8 step headings (NaN = not yet)

    bool worldToGrid(double wx, double wy, CellIndex& cell) const;
    int index(int x, int y) const { return y * width_ + x; }
    int cost(int x, int y) const { return std::max(0, static_cast<int>(map_.data[index(x, y)])); }
    double stepClearance(int x, int y, int dir);  // cached bodyClearance at a cell centre, heading = step dir

    // Smallest bodyClearance turning `turn` rad on the spot from heading
    // `from`, one way round. Gives up once it drops below `floor`.
    double sweepClearance(double x, double y, double from, double turn, double floor) const;
    // True if the robot can turn on the spot at (x, y) from heading `from` to
    // `to`, one way round or the other, keeping `need` m clear all the way
    bool canTurn(double x, double y, double from, double to, double need) const;
};

}

#endif
