#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "planner_core.hpp"

// Everything here is checked against the exact shapes, not the planner's own
// map, so the tests catch the planner fooling itself as well as bugs in it.

namespace {

// The robot's parts as (x0, x1, y0, y1) rectangles, in the frame of the middle
// of its wheel axle (x forward, y left), from robot_env.sdf
const double kParts[3][4] = {
  {-0.5, 1.5, -0.5, 0.5},   // chassis
  {-0.4, 0.4, 0.5, 0.7},    // left wheel
  {-0.4, 0.4, -0.7, -0.5},  // right wheel
};

// Anything the planner plans past should end up at least this far from the body
constexpr double kMinGap = 0.2;

struct Box {
  double x0, x1, y0, y1;
};

double toBox(const Box& b, double x, double y) {
  return std::hypot(std::max({b.x0 - x, 0.0, x - b.x1}), std::max({b.y0 - y, 0.0, y - b.y1}));
}

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double yawOf(const geometry_msgs::msg::PoseStamped& p) {
  return 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
}

// Points every 2 cm round the outside of each part of the body
std::vector<std::pair<double, double>> outline() {
  std::vector<std::pair<double, double>> pts;
  for (const auto& p : kParts) {
    const int nx = static_cast<int>(std::lround((p[1] - p[0]) / 0.02));
    const int ny = static_cast<int>(std::lround((p[3] - p[2]) / 0.02));
    for (int i = 0; i <= nx; ++i) {
      pts.push_back({p[0] + (p[1] - p[0]) * i / nx, p[2]});
      pts.push_back({p[0] + (p[1] - p[0]) * i / nx, p[3]});
    }
    for (int j = 1; j < ny; ++j) {
      pts.push_back({p[0], p[2] + (p[3] - p[2]) * j / ny});
      pts.push_back({p[1], p[2] + (p[3] - p[2]) * j / ny});
    }
  }
  return pts;
}

// A world of boxes, and the map the robot would build of it
class World {
  public:
    World(double x0, double y0, double w, double h, std::vector<Box> boxes) : boxes_(std::move(boxes)) {
      // 0.1 m cells. The lidar only sees surfaces, so the cells just inside
      // each box are hits (100) and the rest of its inside stays unknown
      // (-1); around them the cost fades out over 1.6 m like the costmap's.
      map_.header.frame_id = "sim_world";
      map_.info.resolution = 0.1;
      map_.info.width = static_cast<unsigned>(std::lround(w / 0.1));
      map_.info.height = static_cast<unsigned>(std::lround(h / 0.1));
      map_.info.origin.position.x = x0;
      map_.info.origin.position.y = y0;
      map_.data.assign(map_.info.width * map_.info.height, 0);
      for (unsigned j = 0; j < map_.info.height; ++j) {
        for (unsigned i = 0; i < map_.info.width; ++i) {
          const double x = x0 + (i + 0.5) * 0.1;
          const double y = y0 + (j + 0.5) * 0.1;
          int8_t& cell = map_.data[j * map_.info.width + i];
          const double d = distance(x, y);
          if (d == 0.0) {
            const bool surface = distance(x + 0.1, y) > 0 || distance(x - 0.1, y) > 0 ||
                                 distance(x, y + 0.1) > 0 || distance(x, y - 0.1) > 0;
            cell = surface ? 100 : -1;
          } else if (d < 1.6) {
            cell = static_cast<int8_t>(100 * (1 - d / 1.6));
          }
        }
      }
      planner_.setMap(map_);
    }

    double distance(double x, double y) const {
      double best = 1e9;
      for (const auto& b : boxes_) best = std::min(best, toBox(b, x, y));
      return best;
    }

    // True gap between the body with its axle at (x, y) facing yaw and the nearest box
    double gap(double x, double y, double yaw) const {
      static const auto pts = outline();
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      double best = 1e9;
      for (const auto& p : pts) best = std::min(best, distance(x + c * p.first - s * p.second, y + s * p.first + c * p.second));
      return best;
    }

    // True gap turning on the spot from one heading to another, the better way round
    double turnGap(double x, double y, double from, double to) const {
      const double turn = wrap(to - from);
      double best = -1e9;
      for (double way : {turn, turn - std::copysign(2.0 * M_PI, turn)}) {
        const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(way) / (2.0 * M_PI / 180))));
        double worst = 1e9;
        for (int k = 1; k <= steps; ++k) worst = std::min(worst, gap(x, y, from + way * k / steps));
        best = std::max(best, worst);
      }
      return best;
    }

    // Tightest the body gets anywhere along a path, turning on the spot at the
    // start and wherever the heading jumps included
    double pathGap(const nav_msgs::msg::Path& path, double start_yaw) const {
      double worst = 1e9;
      double last = start_yaw;
      for (const auto& p : path.poses) {
        const double yaw = yawOf(p);
        worst = std::min(worst, gap(p.pose.position.x, p.pose.position.y, yaw));
        if (std::abs(wrap(yaw - last)) > 0.3) {
          worst = std::min(worst, turnGap(p.pose.position.x, p.pose.position.y, last, yaw));
        }
        last = yaw;
      }
      return worst;
    }

    robot::PlannerCore planner_{rclcpp::get_logger("planner_test")};
    nav_msgs::msg::OccupancyGrid map_;

  private:
    std::vector<Box> boxes_;
};

// Outer walls round [x0, x1] x [y0, y1], 0.3 m thick, inside the area
std::vector<Box> walls(double x0, double y0, double x1, double y1) {
  return {{x0, x1, y0, y0 + 0.3}, {x0, x1, y1 - 0.3, y1}, {x0, x0 + 0.3, y0, y1}, {x1 - 0.3, x1, y0, y1}};
}

std::vector<Box> operator+(std::vector<Box> a, const std::vector<Box>& b) {
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

}  // namespace

TEST(BodyCircles, CoverTheWholeBody) {
  const auto circles = robot::bodyCircles();
  for (const auto& p : kParts) {
    for (double x = p[0]; x <= p[1] + 1e-9; x += 0.01) {
      for (double y = p[2]; y <= p[3] + 1e-9; y += 0.01) {
        bool covered = false;
        for (const auto& c : circles) covered = covered || std::hypot(x - c.x, y - c.y) <= c.r + 1e-9;
        EXPECT_TRUE(covered) << "(" << x << ", " << y << ") isn't inside any circle";
      }
    }
  }
}

TEST(PlannerCore, DrivesStraightAcrossOpenFloor) {
  World world(0, 0, 20, 12, walls(0, 0, 20, 12));
  nav_msgs::msg::Path path;
  double gx = 15.0, gy = 8.0;
  const double heading = std::atan2(gy - 3.0, gx - 3.0);
  ASSERT_EQ(world.planner_.planPath(3.0, 3.0, heading, gx, gy, path), robot::PlannerCore::Result::OK);
  // The grid's staircase gets straightened into one line from the robot to the goal
  for (const auto& p : path.poses) EXPECT_NEAR(wrap(yawOf(p) - heading), 0.0, 0.03);
  EXPECT_LT(std::hypot(path.poses.back().pose.position.x - 15.0, path.poses.back().pose.position.y - 8.0), 0.35);
}

TEST(PlannerCore, RoundsCornersItHasRoomFor) {
  // An L-shaped hall 5.7 m wide: east along the bottom, then north up the right side
  World world(0, 0, 20, 20, walls(0, 0, 20, 20) + std::vector<Box>{{0.3, 14.0, 6.0, 19.7}});
  nav_msgs::msg::Path path;
  double gx = 17.0, gy = 17.0;
  ASSERT_EQ(world.planner_.planPath(3.0, 3.0, 0.0, gx, gy, path), robot::PlannerCore::Result::OK);
  // Plenty of room, so the corner is an arc to drive round, not a stop to turn on the spot
  for (size_t i = 1; i < path.poses.size(); ++i) {
    EXPECT_LT(std::abs(wrap(yawOf(path.poses[i]) - yawOf(path.poses[i - 1]))), 0.3)
        << "the heading jumps at (" << path.poses[i].pose.position.x << ", " << path.poses[i].pose.position.y << ")";
  }
  EXPECT_GE(world.pathGap(path, 0.0), kMinGap);
}

TEST(PlannerCore, KeepsTheWholeBodyClear) {
  // A small warehouse: a long shelf, a wall with a 3 m door, a pillar, crates
  const auto boxes = walls(0, 0, 24, 16) + std::vector<Box>{
    {4.0, 14.0, 7.5, 8.5},    // shelf
    {16.0, 16.3, 0.3, 6.0},   // wall, south of the door
    {16.0, 16.3, 9.0, 15.7},  // wall, north of the door
    {8.0, 9.0, 2.5, 3.5},     // pillar
    {19.0, 20.5, 11.0, 12.5}, // crate
    {3.0, 5.0, 11.0, 13.0},   // crate
  };
  World world(0, 0, 24, 16, boxes);
  std::mt19937 rng(7);
  auto uniform = [&](double lo, double hi) { return lo + (hi - lo) * (rng() / 4294967296.0); };
  int trips = 0;
  while (trips < 40) {
    const double sx = uniform(1, 23), sy = uniform(1, 15), syaw = uniform(-M_PI, M_PI);
    double gx = uniform(1, 23), gy = uniform(1, 15);
    // Start somewhere the robot could be, and aim for somewhere it fits
    if (world.gap(sx, sy, syaw) < 0.4 || std::hypot(gx - sx, gy - sy) < 3.0) continue;
    bool fits = false;
    for (int d = 0; d < 8; ++d) fits = fits || world.gap(gx, gy, d * M_PI / 4) > 0.5;
    if (!fits) continue;
    ++trips;
    const double want_x = gx, want_y = gy;
    nav_msgs::msg::Path path;
    ASSERT_EQ(world.planner_.planPath(sx, sy, syaw, gx, gy, path), robot::PlannerCore::Result::OK)
        << "from (" << sx << ", " << sy << ") to (" << want_x << ", " << want_y << ")";
    EXPECT_GE(world.pathGap(path, syaw), kMinGap)
        << "from (" << sx << ", " << sy << ", " << syaw << ") to (" << want_x << ", " << want_y << ")";
    EXPECT_LT(std::hypot(path.poses.back().pose.position.x - want_x, path.poses.back().pose.position.y - want_y), 1.5);
  }
}

TEST(PlannerCore, OnlyTurnsOnTheSpotWhereItHasRoom) {
  // The bottom of the A, T and O in the watonomous world, and a goal just
  // below the 2.3 m gap between the A and the T: too narrow to turn round in
  // with the front of the robot poking into it. Without checking turns on the
  // spot, the planner zig-zags its way up to the goal, turning on the spot
  // under the gap with 0.08 m to spare.
  const std::vector<Box> boxes = {
    {-20.0, -2.0, -10.25, -9.75},         // the hall's south wall
    {-18.008, -17.658, -2.675, 2.675},    // W, right leg
    {-17.008, -16.658, -2.675, 2.0083},   // A, left leg
    {-17.0086, -13.3254, -0.5083, -0.1583},  // A, crossbar
    {-16.342, -13.992, 2.325, 2.675},     // A, top
    {-13.675, -13.325, -2.675, 2.0083},   // A, right leg
    {-12.6746, -8.9914, 2.325, 2.675},    // T, top
    {-11.008, -10.658, -2.675, 2.675},    // T, stem
    {-8.3417, -7.9917, -1.8417, 1.8417},  // O, left
    {-7.5084, -5.4916, -2.675, -2.325},   // O, bottom
    {-7.5084, -5.4916, 2.325, 2.675},     // O, top
    {-5.0083, -4.6583, -1.8417, 1.8417},  // O, right
  };
  World world(-20, -11, 18, 15, boxes);
  nav_msgs::msg::Path path;
  double gx = -12.49, gy = -3.33;
  ASSERT_EQ(world.planner_.planPath(-8.87, -7.43, -1.98, gx, gy, path), robot::PlannerCore::Result::OK);
  EXPECT_GE(world.pathGap(path, -1.98), kMinGap);
}

TEST(PlannerCore, BacksOutOfADeadEnd) {
  // Facing the closed end of a corridor too narrow to turn round in, with the
  // goal back the way it came. The only way is to back out, which the
  // controller does when it has no room to turn, so the planner has to plan
  // as if it could turn rather than give up.
  const auto boxes = walls(0, 0, 20, 10) + std::vector<Box>{
    {10.0, 16.0, 3.5, 3.8},   // corridor's south wall
    {10.0, 16.0, 6.4, 6.7},   // north wall, 2.6 m apart
    {16.0, 16.3, 3.5, 6.7},   // closed end
  };
  World world(0, 0, 20, 10, boxes);
  nav_msgs::msg::Path path;
  double gx = 4.0, gy = 5.0;
  ASSERT_EQ(world.planner_.planPath(13.0, 5.1, 0.0, gx, gy, path), robot::PlannerCore::Result::OK);
  EXPECT_LT(std::hypot(path.poses.back().pose.position.x - 4.0, path.poses.back().pose.position.y - 5.0), 0.35);
}

TEST(PlannerCore, StopsShortOfAGoalInsideSomething) {
  World world(0, 0, 20, 12, walls(0, 0, 20, 12) + std::vector<Box>{{9.5, 10.5, 5.5, 6.5}, {3.0, 7.0, 3.0, 9.0}});
  nav_msgs::msg::Path path;
  // Just inside a 1 m crate: get as close as the body allows
  double gx = 10.3, gy = 6.0;
  ASSERT_EQ(world.planner_.planPath(15.0, 6.0, M_PI, gx, gy, path), robot::PlannerCore::Result::OK);
  EXPECT_LT(std::hypot(gx - 10.3, gy - 6.0), 1.5);
  EXPECT_GT(world.distance(gx, gy), 0.0);
  EXPECT_GE(world.pathGap(path, M_PI), kMinGap);
  // In the middle of a 4 x 6 m block: nowhere near it is reachable
  gx = 5.0;
  gy = 6.0;
  EXPECT_EQ(world.planner_.planPath(15.0, 6.0, M_PI, gx, gy, path), robot::PlannerCore::Result::GOAL_INVALID);
}

// Two poses at the same spot: the path turns on the spot from one to the other
nav_msgs::msg::Path turnOnTheSpot(double x, double y, double from, double to) {
  nav_msgs::msg::Path path;
  for (double yaw : {from, to}) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw / 2);
    p.pose.orientation.w = std::cos(yaw / 2);
    path.poses.push_back(p);
  }
  return path;
}

TEST(PlannerCore, PathClearanceCountsTurningOnTheSpot) {
  // Facing east under a wall and turning to face south. The long way round
  // swings the front into the wall; the short way is fine, and that's the
  // way the robot goes.
  World under_wall(0, 0, 10, 10, walls(0, 0, 10, 10) + std::vector<Box>{{2.0, 8.0, 6.0, 6.3}});
  EXPECT_GT(under_wall.planner_.pathClearance(turnOnTheSpot(4.0, 4.6, 0.0, -M_PI / 2), 0), 0.3);

  // Turning round in a corridor 2.6 m wide: facing either way the body fits
  // with room to spare, but the front corners swing out 1.6 m either way round
  World corridor(0, 0, 10, 10, walls(0, 0, 10, 10) + std::vector<Box>{{1.0, 9.0, 3.0, 3.3}, {1.0, 9.0, 5.9, 6.2}});
  EXPECT_GT(corridor.planner_.bodyClearance(5.0, 4.6, 0.0), 0.3);
  EXPECT_GT(corridor.planner_.bodyClearance(5.0, 4.6, M_PI), 0.3);
  EXPECT_LT(corridor.planner_.pathClearance(turnOnTheSpot(5.0, 4.6, 0.0, M_PI), 0), 0.0);
}
