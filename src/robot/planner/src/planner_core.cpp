#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace robot
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// The 8 steps to a neighbouring cell. Step d heads at d * 45 degrees, so its
// index doubles as a heading for the clearance cache.
const int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
const int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

// Corners are rounded with the biggest of these radii (m) the body fits along.
// Bigger arcs are faster to drive; smaller ones stay closer to the grid path.
const double kArcRadii[] = {2.5, 1.8, 1.2, 0.8};
constexpr double kStraight = 5.0 * M_PI / 180.0;  // turns smaller than this aren't corners
constexpr double kTurnStep = 5.0 * M_PI / 180.0;  // turning on the spot is checked this often
constexpr double kPivot = 0.3;  // rad; a path's heading jumping more than this between poses is a turn on the spot
constexpr int kLookPast = 30;   // cells to keep looking for a shortcut past the first one out of sight

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// The step d with kDx[d], kDy[d] = dx, dy
int stepDirection(int dx, int dy) {
  for (int d = 0; d < 8; ++d) {
    if (kDx[d] == dx && kDy[d] == dy) return d;
  }
  return 0;
}

// Squared distance transform of one row or column (Felzenszwalb and
// Huttenlocher's lower envelope of parabolas): out[q] = min over p of
// (q - p)^2 + f[p]. v and z are scratch space of size n and n + 1.
void distance1D(const std::vector<double>& f, std::vector<double>& out, int n,
                std::vector<int>& v, std::vector<double>& z) {
  int k = 0;
  v[0] = 0;
  z[0] = -1e20;
  z[1] = 1e20;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e20;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    out[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

std::vector<Circle> bodyCircles() {
  std::vector<Circle> circles;
  // Chassis: cut into eight 0.5 m squares, each covered by the circle through
  // its corners (so the circles poke out at most 0.1 m past the chassis)
  const double r = std::hypot(0.25, 0.25);
  for (double x : {-0.25, 0.25, 0.75, 1.25}) {
    for (double y : {-0.25, 0.25}) circles.push_back({x, y, r});
  }
  // Wheels: 0.8 x 0.2 m at y = +-0.6, two circles each
  const double rw = std::hypot(0.2, 0.1);
  for (double x : {-0.2, 0.2}) {
    for (double y : {-0.6, 0.6}) circles.push_back({x, y, rw});
  }
  return circles;
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger), circles_(bodyCircles()) {
  for (const auto& c : circles_) reach_ = std::max(reach_, std::hypot(c.x, c.y) + c.r);
}

bool PlannerCore::worldToGrid(double wx, double wy, CellIndex& cell) const {
  cell.x = static_cast<int>(std::floor((wx - origin_x_) / res_));
  cell.y = static_cast<int>(std::floor((wy - origin_y_) / res_));
  return cell.x >= 0 && cell.x < width_ && cell.y >= 0 && cell.y < height_;
}

void PlannerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
  map_ = map;
  width_ = static_cast<int>(map.info.width);
  height_ = static_cast<int>(map.info.height);
  res_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  const int n = width_ * height_;
  if (n == 0 || res_ <= 0.0 || map.data.size() != static_cast<size_t>(n)) {
    RCLCPP_WARN(logger_, "Map is empty or malformed");
    dist_.clear();
    return;
  }

  // Exact distance from every cell to the nearest lidar hit: the 2D transform
  // is the 1D one down every column, then along every row.
  constexpr double kFar = 1e10;
  std::vector<double> grid(n);
  for (int i = 0; i < n; ++i) grid[i] = map.data[i] >= 100 ? 0.0 : kFar;
  const int longest = std::max(width_, height_);
  std::vector<double> f(longest), out(longest), z(longest + 1);
  std::vector<int> v(longest);
  for (int x = 0; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) f[y] = grid[index(x, y)];
    distance1D(f, out, height_, v, z);
    for (int y = 0; y < height_; ++y) grid[index(x, y)] = out[y];
  }
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) f[x] = grid[index(x, y)];
    distance1D(f, out, width_, v, z);
    for (int x = 0; x < width_; ++x) grid[index(x, y)] = out[x];
  }
  dist_.resize(n);
  for (int i = 0; i < n; ++i) {
    dist_[i] = grid[i] >= kFar / 2 ? 1e6f : static_cast<float>(std::sqrt(grid[i]) * res_);
  }
  clearance_.assign(static_cast<size_t>(n) * 8, std::numeric_limits<float>::quiet_NaN());
}

double PlannerCore::bodyClearance(double x, double y, double yaw) const {
  if (dist_.empty()) return -kInf;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  double best = kInf;
  for (const auto& circle : circles_) {
    CellIndex cell;
    if (!worldToGrid(x + c * circle.x - s * circle.y, y + s * circle.x + c * circle.y, cell)) {
      return -kInf;  // part of the body would be off the map
    }
    best = std::min(best, dist_[index(cell.x, cell.y)] - circle.r);
  }
  return best;
}

double PlannerCore::sweepClearance(double x, double y, double from, double turn, double floor) const {
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(turn) / kTurnStep)));
  double worst = kInf;
  for (int s = 1; s <= steps && worst >= floor; ++s) {
    worst = std::min(worst, bodyClearance(x, y, from + turn * s / steps));
  }
  return worst;
}

double PlannerCore::turnClearance(double x, double y, double from, double to) const {
  const double turn = wrap(to - from);
  const double one_way = sweepClearance(x, y, from, turn, -kInf);
  return std::max(one_way, sweepClearance(x, y, from, turn - std::copysign(2.0 * M_PI, turn), one_way));
}

bool PlannerCore::canTurn(double x, double y, double from, double to, double need) const {
  CellIndex cell;
  if (dist_.empty() || !worldToGrid(x, y, cell)) return false;
  // Nothing within reach of the body whichever way it faces. (The 1.5 cells
  // allow for neither this point nor the circles' centres being on the cell
  // centres that distances are measured between.)
  if (dist_[index(cell.x, cell.y)] - reach_ - 1.5 * res_ >= need) return true;
  const double turn = wrap(to - from);
  return sweepClearance(x, y, from, turn, need) >= need ||
         sweepClearance(x, y, from, turn - std::copysign(2.0 * M_PI, turn), need) >= need;
}

double PlannerCore::pathClearance(const nav_msgs::msg::Path& path, size_t from) const {
  double worst = kInf;
  double last_yaw = 0.0;
  for (size_t i = from; i < path.poses.size(); ++i) {
    const auto& pose = path.poses[i].pose;
    const double yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
    worst = std::min(worst, bodyClearance(pose.position.x, pose.position.y, yaw));
    if (i > from && std::abs(wrap(yaw - last_yaw)) > kPivot) {
      worst = std::min(worst, turnClearance(pose.position.x, pose.position.y, last_yaw, yaw));
    }
    last_yaw = yaw;
  }
  return worst;
}

double PlannerCore::stepClearance(int x, int y, int dir) {
  float& c = clearance_[static_cast<size_t>(index(x, y)) * 8 + dir];
  if (std::isnan(c)) {
    c = static_cast<float>(bodyClearance(origin_x_ + (x + 0.5) * res_, origin_y_ + (y + 0.5) * res_,
                                         dir * M_PI / 4.0));
  }
  return c;
}

PlannerCore::Result PlannerCore::planPath(double start_x, double start_y, double start_yaw,
                                          double& goal_x, double& goal_y, nav_msgs::msg::Path& path) {
  path.header.frame_id = map_.header.frame_id;
  path.poses.clear();
  if (dist_.empty()) {
    RCLCPP_WARN(logger_, "No map to plan on");
    return Result::NO_PATH;
  }

  CellIndex start, goal;
  if (!worldToGrid(start_x, start_y, start)) return Result::START_INVALID;
  if (!worldToGrid(goal_x, goal_y, goal)) return Result::GOAL_INVALID;

  auto cellX = [&](int x) { return origin_x_ + (x + 0.5) * res_; };
  auto cellY = [&](int y) { return origin_y_ + (y + 0.5) * res_; };

  // Room the body needs at a point. Normally body_margin. If the robot is
  // already tighter than that (the map filled in around it, or it was pushed),
  // near it anything no tighter than its current pose is allowed, less a
  // little slack because the grid's headings don't match its exact one. That
  // way it can always drive out, but never towards what it's next to.
  const double margin = params_.body_margin;
  const double start_clear = bodyClearance(start_x, start_y, start_yaw);
  const double escape_sq = params_.escape_radius * params_.escape_radius;
  auto need = [&](double x, double y) {
    if (start_clear >= margin) return margin;
    const double dx = x - start_x;
    const double dy = y - start_y;
    return dx * dx + dy * dy <= escape_sq ? std::min(margin, start_clear - 0.05) : margin;
  };
  auto cellNeed = [&](int x, int y) { return need(cellX(x), cellY(y)); };

  // The robot turns on the spot before driving off if the path starts out
  // pointing another way, so a first step is only any use if it can turn to
  // face it there (either way round) without getting tighter than it has to.
  bool turnable[8];
  for (int d = 0; d < 8; ++d) {
    const double to = std::atan2(cellY(start.y + kDy[d]) - start_y, cellX(start.x + kDx[d]) - start_x);
    turnable[d] = canTurn(start_x, start_y, start_yaw, to, need(start_x, start_y));
  }

  // Octile distance (m): exact cost of an 8-connected move on free cells, and
  // never an overestimate since every step costs at least its length
  auto heuristic = [&](int x, int y) {
    const double dx = std::abs(x - goal.x);
    const double dy = std::abs(y - goal.y);
    return res_ * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
  };

  // Anywhere within arrive_radius of the goal will do. Right next to a wall
  // the body only fits facing some ways, and insisting on the exact cell can
  // mean a long loop round to arrive facing one of those. If nowhere that
  // close can be reached, the search runs out and we take the reachable cell
  // nearest the goal instead, as long as it's within goal_search_radius.
  const double arrive_cells = params_.arrive_radius / res_;
  const int n_cells = width_ * height_;
  const int start_index = index(start.x, start.y);
  std::vector<double> g_score(n_cells);
  std::vector<int> came_from(n_cells);
  std::vector<char> closed(n_cells);
  int end_index = -1;
  int expanded = 0;
  int nearest_index = start_index;
  double nearest = 0.0;
  auto search = [&](const bool* first_steps) {
    std::fill(g_score.begin(), g_score.end(), kInf);
    std::fill(came_from.begin(), came_from.end(), -1);
    std::fill(closed.begin(), closed.end(), 0);
    end_index = -1;
    expanded = 0;
    nearest_index = start_index;
    nearest = std::hypot(start.x - goal.x, start.y - goal.y);
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
    g_score[start_index] = 0.0;
    open.emplace(start, heuristic(start.x, start.y), heuristic(start.x, start.y));
    while (!open.empty()) {
      const CellIndex current = open.top().index;
      open.pop();
      const int ci = index(current.x, current.y);
      if (closed[ci]) continue;  // stale entry, already expanded with a lower cost
      closed[ci] = 1;
      ++expanded;
      const double to_goal = std::hypot(current.x - goal.x, current.y - goal.y);
      if (to_goal <= arrive_cells) {
        end_index = ci;
        return;
      }
      if (to_goal < nearest) {
        nearest = to_goal;
        nearest_index = ci;
      }
      const double here_x = cellX(current.x);
      const double here_y = cellY(current.y);
      const double here_need = need(here_x, here_y);
      // The way the robot faces when it gets here
      double facing = start_yaw;
      if (ci != start_index) {
        const int from = came_from[ci];
        facing = from == start_index ? std::atan2(here_y - start_y, here_x - start_x)
                                     : stepDirection(current.x - from % width_, current.y - from / width_) * M_PI / 4.0;
      }
      for (int d = 0; d < 8; ++d) {
        const int nx = current.x + kDx[d];
        const int ny = current.y + kDy[d];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
        const int ni = index(nx, ny);
        if (closed[ni]) continue;
        // The body must fit at both ends of the step, facing along it (at the
        // start, where the robot turns on the spot first: turn to face it)
        if (stepClearance(nx, ny, d) < cellNeed(nx, ny)) continue;
        if (ci == start_index ? !first_steps[d] : stepClearance(current.x, current.y, d) < here_need) continue;
        // Step length (m), scaled up by the cost of the cell we step into
        const double step = (d % 2 ? std::sqrt(2.0) : 1.0) * res_;
        const double tentative_g = g_score[ci] + step * (1.0 + params_.cost_weight * cost(nx, ny) / 100.0);
        if (tentative_g >= g_score[ni]) continue;
        // A sharp change of direction can only be made by turning on the
        // spot, so there has to be room to. (A staircase along a slanting line
        // zigzags 45 degrees either way, which the shortcuts below straighten
        // out, so those don't count.)
        const double to = d * M_PI / 4.0;
        if (ci != start_index && std::abs(wrap(to - facing)) > M_PI / 3 &&
            !canTurn(here_x, here_y, facing, to, here_need)) {
          continue;
        }
        g_score[ni] = tentative_g;
        came_from[ni] = ci;
        const double h = heuristic(nx, ny);
        open.emplace(CellIndex(nx, ny), tentative_g + h, h);
      }
    }
  };

  search(turnable);
  // If that doesn't get there, maybe the only way on starts in a direction
  // the robot can't turn to where it stands: it's driven into a dead end,
  // say. Then plan as if it could turn any way. The controller finds there's
  // no room to turn and backs up, and again after each replan, until there
  // is. If that doesn't get there either, go with the first answer, unless
  // the robot couldn't get anywhere at all that way.
  bool turn_anyway = false;  // planned as if the robot could turn any way where it stands
  if (end_index < 0 && !std::all_of(std::begin(turnable), std::end(turnable), [](bool t) { return t; })) {
    std::vector<int> first_came_from(came_from);
    const int first_expanded = expanded;
    const int first_nearest_index = nearest_index;
    const double first_nearest = nearest;
    const bool anyway[8] = {true, true, true, true, true, true, true, true};
    search(anyway);
    turn_anyway = end_index >= 0 || first_expanded < 100;
    if (!turn_anyway) {
      came_from.swap(first_came_from);
      expanded = first_expanded;
      nearest_index = first_nearest_index;
      nearest = first_nearest;
    }
  }
  if (end_index < 0) {
    // Barely got anywhere: the robot itself is hemmed in, which may not last
    if (expanded < 100) return Result::NO_PATH;
    if (nearest * res_ > params_.goal_search_radius) return Result::GOAL_INVALID;
    end_index = nearest_index;
  }
  // Report where the path really ends: the goal itself, or the cell we stop at
  if (end_index != index(goal.x, goal.y)) {
    goal_x = cellX(end_index % width_);
    goal_y = cellY(end_index / width_);
  }

  // Walk back from the end to get the cells in order
  std::vector<int> cells;
  for (int i = end_index; i != -1; i = came_from[i]) cells.push_back(i);
  std::reverse(cells.begin(), cells.end());
  if (cells.size() == 1) cells.push_back(cells.front());

  // Cell centres, but starting exactly at the robot and ending exactly where we stop
  std::vector<double> px, py, pcost;
  for (int c : cells) {
    px.push_back(cellX(c % width_));
    py.push_back(cellY(c / width_));
    pcost.push_back(std::max(0, static_cast<int>(map_.data[c])));
  }
  px.front() = start_x;  py.front() = start_y;
  px.back() = goal_x;    py.back() = goal_y;

  // True if the body fits all along the straight line a -> b facing along it,
  // and the line never crosses a cell costlier than max_cost
  auto lineClear = [&](size_t a, size_t b, double max_cost) {
    const double dx = px[b] - px[a];
    const double dy = py[b] - py[a];
    const double heading = std::atan2(dy, dx);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / (res_ * 0.5))));
    for (int s = 0; s <= steps; ++s) {
      const double x = px[a] + dx * s / steps;
      const double y = py[a] + dy * s / steps;
      CellIndex c;
      if (!worldToGrid(x, y, c) || cost(c.x, c.y) > max_cost) return false;
      if (bodyClearance(x, y, heading) < need(x, y)) return false;
    }
    return true;
  };

  // Corners. A path of straight lines makes the controller cut every corner
  // (it steers at a point ahead of it), so each one is rounded into an arc
  // the robot can actually drive, checking the body all the way round: the
  // front swings out and the side swings in. If no arc fits, the corner stays
  // sharp and the robot stops there and turns on the spot, if it has room to.
  struct Corner {
    double radius = 0.0;  // 0 = turn on the spot at the waypoint
    double tangent = 0.0; // distance from the waypoint to where the arc starts/ends
    double cx = 0.0, cy = 0.0, side = 0.0;  // arc centre, +1 turning left, -1 right
  };
  // Point on a corner's arc where the path heads at angle a
  auto arcPoint = [](const Corner& c, double a, double& x, double& y) {
    x = c.cx + c.side * c.radius * std::sin(a);
    y = c.cy - c.side * c.radius * std::cos(a);
  };
  // The corner at (x, y) from heading h_in to h_out, with room_in and
  // room_out of straight line either side of it for an arc to start and end
  // in. False if the robot can neither drive round it nor turn on the spot.
  auto fitCorner = [&](double x, double y, double h_in, double h_out, double room_in, double room_out,
                       Corner& corner) {
    corner = Corner();
    const double turn = wrap(h_out - h_in);
    if (std::abs(turn) < kStraight) return true;
    double best_clear = -kInf;
    for (double radius : kArcRadii) {
      Corner c;
      c.radius = radius;
      c.tangent = radius * std::tan(std::abs(turn) / 2);
      if (c.tangent > room_in || c.tangent > room_out) continue;
      c.side = turn > 0 ? 1.0 : -1.0;
      const double sx = x - c.tangent * std::cos(h_in);
      const double sy = y - c.tangent * std::sin(h_in);
      c.cx = sx - c.side * radius * std::sin(h_in);
      c.cy = sy + c.side * radius * std::cos(h_in);
      // Walk round the arc every 0.1 m (or 5 degrees), facing along it
      const int steps = std::max(2, static_cast<int>(std::ceil(std::max(std::abs(turn) * radius / res_,
                                                                        std::abs(turn) / (M_PI / 36)))));
      double clear = kInf;
      for (int s = 0; s <= steps && clear >= margin; ++s) {
        const double a = h_in + turn * s / steps;
        double ax, ay;
        arcPoint(c, a, ax, ay);
        clear = std::min(clear, bodyClearance(ax, ay, a) - need(ax, ay) + margin);
      }
      if (clear < margin) continue;
      // Take the biggest arc with some room to spare, else the roomiest one that fits
      if (clear >= margin + 0.1) {
        corner = c;
        return true;
      }
      if (clear > best_clear) {
        best_clear = clear;
        corner = c;
      }
    }
    return corner.radius > 0.0 || canTurn(x, y, h_in, h_out, need(x, y));
  };

  // Line-of-sight shortcuts: replace the grid staircase with straight lines,
  // each from the last waypoint to the farthest cell it can see whose corner
  // the robot can get round. A shortcut may not enter cells costlier than the
  // part of the A* path it replaces, so it never cuts closer to obstacles
  // than A* chose to.
  const size_t n = cells.size();
  auto headingTo = [&](size_t a, size_t b) { return std::atan2(py[b] - py[a], px[b] - px[a]); };
  auto lengthTo = [&](size_t a, size_t b) { return std::hypot(px[b] - px[a], py[b] - py[a]); };
  std::vector<size_t> waypoints{0};
  std::vector<Corner> corners{Corner()};  // corners[k] is the corner at waypoints[k]
  std::vector<size_t> visible;
  while (waypoints.back() + 1 < n) {
    const size_t a = waypoints.back();
    // The body faces along each line, so whether a line is clear depends on
    // its heading, and seeing a cell doesn't mean seeing the ones before it.
    // Just past a bend, lines to the nearest cells are steep and clip what the
    // path bends round; further on they're shallow and clear. So keep looking
    // a way past the first cell out of sight.
    visible.clear();
    double seg_max = pcost[a];
    int unseen = 0;
    for (size_t j = a + 1; j < n && unseen <= kLookPast; ++j) {
      seg_max = std::max(seg_max, pcost[j]);
      if (lineClear(a, j, seg_max)) {
        visible.push_back(j);
        unseen = 0;
      } else {
        ++unseen;
      }
    }
    // The line in, and how much of it the corner before took for its arc
    const size_t prev = waypoints.size() > 1 ? waypoints[waypoints.size() - 2] : 0;
    const double room_in = a > 0 ? lengthTo(prev, a) - corners[corners.size() - 2].tangent : 0.0;
    size_t next = 0;
    Corner corner;
    for (auto it = visible.rbegin(); it != visible.rend() && next == 0; ++it) {
      const size_t j = *it;
      const double h = headingTo(a, j);
      const double len = lengthTo(a, j);
      bool ok;
      if (a == 0) {
        // No corner at the start, but the robot turns there to face the line
        ok = turn_anyway || canTurn(start_x, start_y, start_yaw, h, need(start_x, start_y));
      } else {
        ok = fitCorner(px[a], py[a], headingTo(prev, a), h, room_in, j + 1 == n ? len : len / 2, corner);
      }
      if (ok) next = j;
    }
    if (next == 0) {
      // Nothing works (A* only turns sharply where there's room, so this is rare):
      // follow the grid path one cell on, and turn on the spot here anyway.
      next = a + 1;
      if (a > 0) {
        const double len = lengthTo(a, next);
        fitCorner(px[a], py[a], headingTo(prev, a), headingTo(a, next), room_in, next + 1 == n ? len : len / 2,
                  corner);
      }
      RCLCPP_DEBUG(logger_, "Tight corner at (%.1f, %.1f): no room to turn there", px[a], py[a]);
    }
    corners.back() = corner;
    waypoints.push_back(next);
    corners.push_back(Corner());
  }

  const size_t m = waypoints.size();
  std::vector<double> wx(m), wy(m), heading(m - 1), length(m - 1);
  for (size_t k = 0; k < m; ++k) {
    wx[k] = px[waypoints[k]];
    wy[k] = py[waypoints[k]];
  }
  for (size_t k = 0; k + 1 < m; ++k) {
    heading[k] = std::atan2(wy[k + 1] - wy[k], wx[k + 1] - wx[k]);
    length[k] = std::hypot(wx[k + 1] - wx[k], wy[k + 1] - wy[k]);
  }

  // Lay the path out every cell length, with each pose facing the way the
  // robot drives. Where the robot turns on the spot the heading jumps.
  auto addPose = [&](double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
  };
  for (size_t k = 0; k + 1 < m; ++k) {
    // This line runs from the end of the previous arc to the start of the next
    const double from = corners[k].radius > 0.0 ? corners[k].tangent : 0.0;
    const double to = length[k] - (k + 2 < m && corners[k + 1].radius > 0.0 ? corners[k + 1].tangent : 0.0);
    const double c = std::cos(heading[k]);
    const double s = std::sin(heading[k]);
    const int steps = std::max(1, static_cast<int>(std::ceil((to - from) / res_)));
    for (int i = 0; i < steps; ++i) {
      const double d = from + (to - from) * i / steps;
      addPose(wx[k] + c * d, wy[k] + s * d, heading[k]);
    }
    if (k + 2 > m - 1) continue;  // last line: nothing after it
    const Corner& corner = corners[k + 1];
    if (corner.radius > 0.0) {
      const double turn = wrap(heading[k + 1] - heading[k]);
      const int arc_steps = std::max(2, static_cast<int>(std::ceil(std::abs(turn) * corner.radius / res_)));
      for (int i = 0; i < arc_steps; ++i) {
        const double a = heading[k] + turn * i / arc_steps;
        double x, y;
        arcPoint(corner, a, x, y);
        addPose(x, y, a);
      }
    } else {
      addPose(wx[k + 1], wy[k + 1], heading[k]);  // arrive facing the old way, then turn
    }
  }
  addPose(wx.back(), wy.back(), heading.back());
  return Result::OK;
}

}
