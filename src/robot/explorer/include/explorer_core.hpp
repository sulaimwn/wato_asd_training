#ifndef EXPLORER_CORE_HPP_
#define EXPLORER_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// One patch of the boundary between explored and unexplored space
struct Frontier {
  double goal_x;    // world position the robot should drive to (a frontier cell
  double goal_y;    //   near the middle of the patch)
  double distance;  // m, travel distance to it through known free space
  int size;         // number of frontier cells in the patch
};

class ExplorerCore {
  public:
    struct Params {
      int free_threshold = 50;     // known cells cheaper than this are free
      int frontier_max_cost = 0;   // frontier cells can't cost more than this; see findFrontiers
      int min_frontier_size = 8;   // cells; smaller patches are ignored
      double escape_radius = 1.0;  // m; see findFrontiers
    };

    explicit ExplorerCore(const rclcpp::Logger& logger);
    void setParams(const Params& params) { params_ = params; }

    // Frontiers the robot can reach through known free space, closest first.
    // A frontier cell is a free cell next to an unknown (-1) one, and it must
    // cost no more than frontier_max_cost. With the default of 0 that means a
    // cell the lidar actually saw as empty, away from every obstacle's halo:
    // the costmap also inflates into cells it has never seen (behind walls,
    // inside boxes), and those "known" halo cells next to real unknown space
    // would otherwise look like frontiers. It also keeps frontier goals out of
    // the tight spots right next to obstacles.
    // The robot may be parked inside an inflated zone, so non-lethal cells
    // within escape_radius of it also count as passable when searching outwards.
    // If cells is given, it receives the world position of every frontier cell
    // that belongs to a returned frontier.
    std::vector<Frontier> findFrontiers(const nav_msgs::msg::OccupancyGrid& map,
                                        double robot_x, double robot_y,
                                        std::vector<std::pair<double, double>>* cells = nullptr) const;

    // True if any of the points is within radius of (x, y)
    static bool nearAny(const std::vector<std::pair<double, double>>& points,
                        double x, double y, double radius);

  private:
    rclcpp::Logger logger_;
    Params params_;
};

}

#endif
