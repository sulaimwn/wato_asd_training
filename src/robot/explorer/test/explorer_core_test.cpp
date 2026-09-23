#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "explorer_core.hpp"

// Maps are drawn as strings, one row per string, top row = highest y:
//   '.' free (0)   '#' obstacle (100)   '?' unknown (-1)
//   'i' inflated but passable (20)   'o' inflated and blocked (60)
// Each character is one 1 m cell and the map's origin is (0, 0).
static nav_msgs::msg::OccupancyGrid makeMap(const std::vector<std::string>& rows) {
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "sim_world";
  map.info.resolution = 1.0;
  map.info.width = rows.front().size();
  map.info.height = rows.size();
  map.data.resize(map.info.width * map.info.height);
  for (size_t r = 0; r < rows.size(); ++r) {
    size_t y = rows.size() - 1 - r;
    for (size_t x = 0; x < rows[r].size(); ++x) {
      char c = rows[r][x];
      int8_t v = c == '.' ? 0 : c == '#' ? 100 : c == 'o' ? 60 : c == 'i' ? 20 : -1;
      map.data[y * map.info.width + x] = v;
    }
  }
  return map;
}

class ExplorerCoreTest : public ::testing::Test {
  protected:
    void SetUp() override {
      robot::ExplorerCore::Params params;
      params.free_threshold = 50;
      params.min_frontier_size = 3;
      params.escape_radius = 1.5;
      explorer_.setParams(params);
    }
    robot::ExplorerCore explorer_{rclcpp::get_logger("explorer_test")};
};

TEST_F(ExplorerCoreTest, FullyKnownMapHasNoFrontiers) {
  auto map = makeMap({
    "#######",
    "#.....#",
    "#.....#",
    "#######",
  });
  EXPECT_TRUE(explorer_.findFrontiers(map, 2.5, 1.5).empty());
}

TEST_F(ExplorerCoreTest, FindsTheEdgeOfTheUnknown) {
  auto map = makeMap({
    "#########",
    "#....????",
    "#....????",
    "#....????",
    "#########",
  });
  std::vector<std::pair<double, double>> cells;
  auto frontiers = explorer_.findFrontiers(map, 1.5, 2.5, &cells);
  ASSERT_EQ(frontiers.size(), 1u);
  // The frontier is the column of free cells touching the unknown (x = 4)
  EXPECT_DOUBLE_EQ(frontiers[0].goal_x, 4.5);
  EXPECT_DOUBLE_EQ(frontiers[0].goal_y, 2.5);  // middle of the column
  EXPECT_EQ(frontiers[0].size, 3);
  EXPECT_DOUBLE_EQ(frontiers[0].distance, 3.0);  // 3 cells from the robot
  EXPECT_EQ(cells.size(), 3u);
}

TEST_F(ExplorerCoreTest, IgnoresFrontiersBehindWalls) {
  // The unknown on the right is only reachable through the wall
  auto map = makeMap({
    "###########",
    "#...#...???",
    "#...#...???",
    "#...#...???",
    "###########",
  });
  EXPECT_TRUE(explorer_.findFrontiers(map, 1.5, 2.5).empty());
}

TEST_F(ExplorerCoreTest, IgnoresTinyFrontiers) {
  // A single unknown cell is ringed by just 4 frontier cells
  robot::ExplorerCore::Params params;
  params.min_frontier_size = 5;
  explorer_.setParams(params);
  auto map = makeMap({
    "#######",
    "#.....#",
    "#..?..#",
    "#.....#",
    "#######",
  });
  EXPECT_TRUE(explorer_.findFrontiers(map, 1.5, 1.5).empty());
}

TEST_F(ExplorerCoreTest, ClosestFrontierComesFirst) {
  auto map = makeMap({
    "###########",
    "???.....???",
    "???.....???",
    "???.....???",
    "###########",
  });
  auto frontiers = explorer_.findFrontiers(map, 3.5, 2.5);  // robot next to the left side
  ASSERT_EQ(frontiers.size(), 2u);
  EXPECT_DOUBLE_EQ(frontiers[0].goal_x, 3.5);
  EXPECT_DOUBLE_EQ(frontiers[1].goal_x, 7.5);
  EXPECT_LT(frontiers[0].distance, frontiers[1].distance);
}

TEST_F(ExplorerCoreTest, RobotInsideInflatedZoneCanStillSearch) {
  // The robot is parked in an inflated (blocked) zone two cells deep. It can
  // only get out because blocked cells within escape_radius count as passable.
  auto map = makeMap({
    "##########",
    "#oo....???",
    "#oo....???",
    "#oo....???",
    "##########",
  });
  auto frontiers = explorer_.findFrontiers(map, 1.5, 2.5);  // escape_radius 1.5 m
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_DOUBLE_EQ(frontiers[0].goal_x, 6.5);

  robot::ExplorerCore::Params params;
  params.min_frontier_size = 3;
  params.escape_radius = 0.5;  // too small to reach the free cells
  explorer_.setParams(params);
  EXPECT_TRUE(explorer_.findFrontiers(map, 1.5, 2.5).empty());
}

TEST_F(ExplorerCoreTest, InflationNextToUnknownIsNotAFrontier) {
  // The costmap inflates obstacles into cells it has never seen (behind a
  // wall, inside a box), so the map can hold a band of "known" low-cost cells
  // right next to real unknown space. Found in testing: those bands ringed the
  // whole building from the outside and got merged into one giant fake
  // frontier. Only cells seen as empty (cost 0) may be frontier cells.
  auto map = makeMap({
    "##########",
    "#....ii???",
    "#....ii???",
    "#....ii???",
    "##########",
  });
  EXPECT_TRUE(explorer_.findFrontiers(map, 1.5, 2.5).empty());

  // With the old rule (any passable cell) the band is reported as a frontier
  robot::ExplorerCore::Params params;
  params.min_frontier_size = 3;
  params.frontier_max_cost = 49;
  explorer_.setParams(params);
  auto frontiers = explorer_.findFrontiers(map, 1.5, 2.5);
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_DOUBLE_EQ(frontiers[0].goal_x, 6.5);
}

TEST_F(ExplorerCoreTest, NearAny) {
  std::vector<std::pair<double, double>> points{{0.0, 0.0}, {10.0, 0.0}};
  EXPECT_TRUE(robot::ExplorerCore::nearAny(points, 9.5, 0.0, 1.0));
  EXPECT_FALSE(robot::ExplorerCore::nearAny(points, 5.0, 0.0, 1.0));
}
