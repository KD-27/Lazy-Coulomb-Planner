// Copyright 2026 Kaveesha Dhananjaya
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lazy_coulomb_planner/lazy_coulomb_planner.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  lazy_coulomb_planner::LazyCoulombPlanner,
  nav2_core::GlobalPlanner)

namespace lazy_coulomb_planner
{

// Clearance waypoints move sideways by at most this fraction of the distance to
// their nearer neighbour (0.35 keeps the turn at the neighbour under about 20 deg).
static const double kMaxOffsetRatio = 0.35;

// While descending the cost gradient, this many consecutive steps without a
// change in cost are tolerated before the descent is considered finished.
static const int kMaxFlatSteps = 2;

// =============================================================================
//  Nav2 Lifecycle Interface
// =============================================================================

void LazyCoulombPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LazyCoulombPlanner: Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node, name_ + "." + key, value);
      return node->get_parameter(name_ + "." + key);
    };

  max_iterations_ = declare("max_iterations", rclcpp::ParameterValue(2000)).as_int();
  max_push_iterations_ = declare("max_push_iterations", rclcpp::ParameterValue(400)).as_int();
  force_model_ = declare("force_model", rclcpp::ParameterValue("coulomb")).as_string();
  force_exponent_ = declare("force_exponent", rclcpp::ParameterValue(2.0)).as_double();
  charge_radius_ = declare("charge_radius", rclcpp::ParameterValue(1.0)).as_double();
  repulsion_strength_ = declare("repulsion_strength", rclcpp::ParameterValue(1.0)).as_double();
  max_push_step_ = declare("max_push_step", rclcpp::ParameterValue(0.0)).as_double();
  force_balance_threshold_ =
    declare("force_balance_threshold", rclcpp::ParameterValue(0.05)).as_double();
  commit_direction_ = declare("commit_direction", rclcpp::ParameterValue(true)).as_bool();
  retry_other_side_ = declare("retry_other_side", rclcpp::ParameterValue(true)).as_bool();
  retry_order_ = declare("retry_order", rclcpp::ParameterValue("shortest")).as_string();
  max_path_stretch_ = declare("max_path_stretch", rclcpp::ParameterValue(5.0)).as_double();
  start_escape_radius_ =
    declare("start_escape_radius", rclcpp::ParameterValue(0.5)).as_double();
  comfort_cost_ = declare("comfort_cost", rclcpp::ParameterValue(100.0)).as_double();
  smoothing_cost_slack_ =
    declare("smoothing_cost_slack", rclcpp::ParameterValue(40.0)).as_double();
  max_clearance_push_ =
    declare("max_clearance_push", rclcpp::ParameterValue(1.0)).as_double();
  waypoint_margin_ = declare("waypoint_margin", rclcpp::ParameterValue(0.3)).as_double();
  simplify_path_ = declare("simplify_path", rclcpp::ParameterValue(true)).as_bool();
  enable_smoothing_ = declare("enable_smoothing", rclcpp::ParameterValue(true)).as_bool();
  smoothing_iterations_ = declare("smoothing_iterations", rclcpp::ParameterValue(5)).as_int();
  output_spacing_ = declare("output_spacing", rclcpp::ParameterValue(0.1)).as_double();
  lethal_cost_threshold_ =
    declare("lethal_cost_threshold", rclcpp::ParameterValue(253.0)).as_double();

  if (retry_order_ != "shortest" && retry_order_ != "recent") {
    throw std::runtime_error(
            "LazyCoulombPlanner: retry_order must be 'shortest' or 'recent', got '" +
            retry_order_ + "'");
  }
  if (force_model_ != "coulomb" && force_model_ != "probe") {
    throw std::runtime_error(
            "LazyCoulombPlanner: force_model must be 'coulomb' or 'probe', got '" +
            force_model_ + "'");
  }

  RCLCPP_INFO(
    logger_,
    "LazyCoulombPlanner configured: force_model=%s, exponent=%.1f, charge_radius=%.2f m",
    force_model_.c_str(), force_exponent_, charge_radius_);
}

void LazyCoulombPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "LazyCoulombPlanner cleaning up");
  costmap_ = nullptr;
}

void LazyCoulombPlanner::activate()
{
  RCLCPP_INFO(logger_, "LazyCoulombPlanner activated");
}

void LazyCoulombPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "LazyCoulombPlanner deactivated");
}

// =============================================================================
//  createPlan
// =============================================================================

nav_msgs::msg::Path LazyCoulombPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  std::function<bool()> cancel_checker)
{
  if (!costmap_) {
    throw nav2_core::PlannerException("LazyCoulombPlanner: Costmap not available");
  }

  PathPoint start(start_pose.pose.position.x, start_pose.pose.position.y, true);
  PathPoint goal(goal_pose.pose.position.x, goal_pose.pose.position.y, true);

  unsigned int mx, my;
  if (!costmap_->worldToMap(start.x, start.y, mx, my)) {
    throw nav2_core::StartOutsideMapBounds("LazyCoulombPlanner: Start is outside costmap");
  }
  if (!costmap_->worldToMap(goal.x, goal.y, mx, my)) {
    throw nav2_core::GoalOutsideMapBounds("LazyCoulombPlanner: Goal is outside costmap");
  }
  // A robot that stopped a little too close to an obstacle sits on a cell the
  // costmap counts as occupied. Rather than refuse to plan, step out to the
  // nearest free cell and plan from there; the real start is put back at the end.
  const PathPoint robot = start;
  const bool escaped = isPointInObstacle(start.x, start.y);
  if (escaped && !nearestFreePoint(robot, start_escape_radius_, start)) {
    throw nav2_core::StartOccupied(
            "LazyCoulombPlanner: Start is inside an obstacle with no free space within " +
            std::to_string(start_escape_radius_) + " m");
  }
  if (isPointInObstacle(goal.x, goal.y)) {
    throw nav2_core::GoalOccupied("LazyCoulombPlanner: Goal is inside an obstacle");
  }

  // Lazy start: the straight line. Segments before `seg` are verified clear.
  std::vector<PathPoint> path{start, goal};
  size_t seg = 0;
  int corrections = 0;

  // Every waypoint could have left its obstacle on either side. The side not
  // taken is remembered here, together with the path as it was, so a branch
  // that dead-ends can be undone and retried the other way.
  struct Alternative
  {
    std::vector<PathPoint> path;
    size_t seg;
    PathPoint entry;
    int side;
    double length;  // length of `path`, used to order retries
  };
  std::vector<Alternative> alternatives;

  const double res = costmap_->getResolution();
  const double max_length =
    max_path_stretch_ * std::hypot(goal.x - start.x, goal.y - start.y) + 20.0 * res;

  // Push `waypoint` out on `side` and splice it into `base` after index `at`.
  // Fails if it cannot be cleared, makes no progress, or the detour gets absurdly long.
  auto tryInsert = [&](
    const std::vector<PathPoint> & base, size_t at, PathPoint waypoint, int side,
    bool forced) -> bool
    {
      const PathPoint & a = base[at];
      const PathPoint & b = base[at + 1];
      const PathPoint entry_point = waypoint;
      if (!pushClear(waypoint, a, b, side, forced)) {
        return false;
      }
      // Keep going past the obstacle edge: at least waypoint_margin, so the corner
      // has room to be rounded off, and then for as long as the costmap cost keeps
      // falling towards comfort_cost. Stop as soon as the cost rises again, which
      // means another obstacle is getting closer (the middle of a gap).
      const double pushed = std::hypot(waypoint.x - entry_point.x, waypoint.y - entry_point.y);
      if (pushed > 1e-9) {
        const double ux = (waypoint.x - entry_point.x) / pushed;
        const double uy = (waypoint.y - entry_point.y) / pushed;
        double cost = costAt(waypoint.x, waypoint.y);
        // Extra clearance must not turn the detour into a hairpin: the sideways
        // offset stays well below the distance to either neighbour.
        const double gentle = kMaxOffsetRatio * std::min(
          std::hypot(entry_point.x - a.x, entry_point.y - a.y),
          std::hypot(entry_point.x - b.x, entry_point.y - b.y));
        PathPoint probe = waypoint;
        int flat_steps = 0;
        for (double d = res; d <= max_clearance_push_ + 1e-9; d += res) {
          probe.x += ux * res;
          probe.y += uy * res;
          const double next_cost = costAt(probe.x, probe.y);
          if (next_cost >= lethal_cost_threshold_ || next_cost > cost) {
            break;
          }
          if (d <= waypoint_margin_ + 1e-9) {
            waypoint = probe;
            cost = next_cost;
            continue;
          }
          if (cost <= comfort_cost_ || pushed + d > gentle) {
            break;
          }
          if (next_cost == cost) {
            // A flat step (same cell again, or equal costs) is not the bottom yet.
            if (++flat_steps > kMaxFlatSteps) {
              break;
            }
            continue;
          }
          waypoint = probe;
          cost = next_cost;
          flat_steps = 0;
        }
      }
      if (std::hypot(waypoint.x - a.x, waypoint.y - a.y) < 0.5 * res ||
        std::hypot(waypoint.x - b.x, waypoint.y - b.y) < 0.5 * res)
      {
        return false;  // landed on a neighbour: the same correction would repeat forever
      }
      double length = 0.0;
      for (size_t i = 0; i + 1 < base.size(); ++i) {
        if (i == at) {
          length += std::hypot(waypoint.x - a.x, waypoint.y - a.y) +
            std::hypot(b.x - waypoint.x, b.y - waypoint.y);
        } else {
          length += std::hypot(base[i + 1].x - base[i].x, base[i + 1].y - base[i].y);
        }
      }
      if (length > max_length) {
        return false;
      }
      waypoint.is_locked = true;
      std::vector<PathPoint> next = base;
      next.insert(next.begin() + at + 1, waypoint);
      path = std::move(next);
      seg = at;  // the new segment (path[at] -> waypoint) is checked next
      return true;
    };

  while (seg + 1 < path.size()) {
    PathPoint entry(0.0, 0.0);
    if (!findSegmentObstacleEntry(path[seg], path[seg + 1], entry)) {
      ++seg;
      continue;
    }

    const int side = preferredSide(entry, path[seg], path[seg + 1]);
    if (retry_other_side_) {
      double length = 0.0;
      for (size_t i = 0; i + 1 < path.size(); ++i) {
        length += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
      }
      alternatives.push_back({path, seg, entry, -side, length});
    }
    bool placed = false;
    bool first_attempt = true;
    while (!placed) {
      if (cancel_checker && cancel_checker()) {
        throw nav2_core::PlannerCancelled("LazyCoulombPlanner: Planning was cancelled");
      }
      if (++corrections > max_iterations_) {
        throw nav2_core::NoValidPathCouldBeFound(
                "LazyCoulombPlanner: Correction budget (" + std::to_string(max_iterations_) +
                ") exhausted");
      }
      if (first_attempt) {
        first_attempt = false;
        placed = tryInsert(path, seg, entry, side, false);
        continue;
      }
      if (alternatives.empty()) {
        throw nav2_core::NoValidPathCouldBeFound(
                "LazyCoulombPlanner: No side of the blocking obstacles leads to a clear path");
      }
      // Dead end: undo back to a waypoint with an untried side.
      // "recent" undoes the latest decision; "shortest" resumes from the shortest
      // partial path, which revisits early decisions sooner.
      size_t pick = alternatives.size() - 1;
      if (retry_order_ == "shortest") {
        for (size_t i = 0; i < alternatives.size(); ++i) {
          if (alternatives[i].length < alternatives[pick].length) {
            pick = i;
          }
        }
      }
      const Alternative alt = std::move(alternatives[pick]);
      alternatives.erase(alternatives.begin() + pick);
      placed = tryInsert(alt.path, alt.seg, alt.entry, alt.side, true);
    }
  }

  if (simplify_path_) {
    path = removeRedundant(path);
  }

  path = improveClearance(path);

  if (enable_smoothing_) {
    for (int i = 0; i < smoothing_iterations_ && path.size() >= 3; ++i) {
      path = chaikinSmooth(path);
    }
  }

  if (output_spacing_ > 0.0) {
    path = densify(path, output_spacing_);
  }

  // Safety net: nothing leaves the planner without a final collision check.
  if (!isPathClear(path)) {
    throw nav2_core::NoValidPathCouldBeFound(
            "LazyCoulombPlanner: Final path failed its collision check");
  }

  if (escaped) {
    path.insert(path.begin(), robot);
  }

  std_msgs::msg::Header header;
  header.stamp = clock_->now();
  header.frame_id = global_frame_;

  RCLCPP_DEBUG(
    logger_, "LCP: %zu poses after %d corrections", path.size(), corrections);

  return toNavPath(path, header, goal_pose.pose.orientation);
}

// =============================================================================
//  Obstacle queries
// =============================================================================

bool LazyCoulombPlanner::isPointInObstacle(double wx, double wy) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return true;
  }
  return static_cast<double>(costmap_->getCost(mx, my)) >= lethal_cost_threshold_;
}

template<typename Visitor>
void LazyCoulombPlanner::traverseCells(
  const PathPoint & p1, const PathPoint & p2, Visitor && visit) const
{
  // Exact grid traversal (Amanatides & Woo): visits every cell the segment
  // touches, so a corner clipped by a short chord cannot be skipped.
  // visit(cost, t_enter, t_exit) gets the cell's cost (255 outside the map) and
  // the stretch of the segment, as a fraction in [0, 1], that lies inside it.
  // Returning true stops the traversal.
  const double res = costmap_->getResolution();
  const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap_->getSizeInCellsY());
  const unsigned char * cells = costmap_->getCharMap();

  const double x0 = (p1.x - costmap_->getOriginX()) / res;
  const double y0 = (p1.y - costmap_->getOriginY()) / res;
  const double dx = (p2.x - p1.x) / res;
  const double dy = (p2.y - p1.y) / res;

  int cx = static_cast<int>(std::floor(x0));
  int cy = static_cast<int>(std::floor(y0));
  const int end_x = static_cast<int>(std::floor(x0 + dx));
  const int end_y = static_cast<int>(std::floor(y0 + dy));

  const int step_x = dx > 0.0 ? 1 : -1;
  const int step_y = dy > 0.0 ? 1 : -1;
  const double inf = std::numeric_limits<double>::infinity();
  // Segment parameter at which the next vertical / horizontal cell edge is crossed.
  const double dt_x = dx != 0.0 ? std::abs(1.0 / dx) : inf;
  const double dt_y = dy != 0.0 ? std::abs(1.0 / dy) : inf;
  double t_next_x = dx != 0.0 ? ((dx > 0.0 ? cx + 1 - x0 : x0 - cx) * dt_x) : inf;
  double t_next_y = dy != 0.0 ? ((dy > 0.0 ? cy + 1 - y0 : y0 - cy) * dt_y) : inf;

  double t_enter = 0.0;
  const int max_cells = std::abs(end_x - cx) + std::abs(end_y - cy) + 1;
  for (int n = 0; n < max_cells; ++n) {
    const double t_exit = std::min({t_next_x, t_next_y, 1.0});
    const bool outside = cx < 0 || cy < 0 || cx >= size_x || cy >= size_y;
    const unsigned char cost = outside ?
      nav2_costmap_2d::NO_INFORMATION : cells[static_cast<size_t>(cy) * size_x + cx];
    if (visit(cost, t_enter, t_exit) || t_exit >= 1.0) {
      return;
    }
    if (t_next_x < t_next_y) {
      cx += step_x;
      t_next_x += dt_x;
    } else {
      cy += step_y;
      t_next_y += dt_y;
    }
    t_enter = t_exit;
  }
}

bool LazyCoulombPlanner::findSegmentObstacleEntry(
  const PathPoint & p1, const PathPoint & p2, PathPoint & intersection) const
{
  bool found = false;
  traverseCells(
    p1, p2, [&](unsigned char cost, double t_enter, double t_exit) {
      if (static_cast<double>(cost) < lethal_cost_threshold_) {
        return false;
      }
      // Middle of the chord through the offending cell: guaranteed to lie inside it.
      const double t = 0.5 * (t_enter + t_exit);
      intersection = PathPoint(p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t);
      found = true;
      return true;
    });
  return found;
}

double LazyCoulombPlanner::segmentMaxCost(
  const PathPoint & p1, const PathPoint & p2, PathPoint & where) const
{
  // `where` is the middle of the stretch between the first and last cell that
  // have the highest cost, so along a wall it lands mid-way, not at one end.
  double worst = -1.0, t_first = 0.0, t_last = 0.0;
  traverseCells(
    p1, p2, [&](unsigned char cost, double t_enter, double t_exit) {
      if (static_cast<double>(cost) > worst) {
        worst = cost;
        t_first = t_enter;
      }
      if (static_cast<double>(cost) == worst) {
        t_last = t_exit;
      }
      return false;
    });
  const double t = 0.5 * (t_first + t_last);
  where = PathPoint(p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t);
  return worst;
}

double LazyCoulombPlanner::segmentExposure(const PathPoint & p1, const PathPoint & p2) const
{
  double fraction = 0.0;
  traverseCells(
    p1, p2, [&](unsigned char cost, double t_enter, double t_exit) {
      if (static_cast<double>(cost) > comfort_cost_) {
        fraction += (t_exit - t_enter) * (cost - comfort_cost_);
      }
      return false;
    });
  return fraction * std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double LazyCoulombPlanner::costAt(double wx, double wy) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  return costmap_->getCost(mx, my);
}

bool LazyCoulombPlanner::segmentIntersectsObstacle(
  const PathPoint & p1, const PathPoint & p2) const
{
  PathPoint unused(0.0, 0.0);
  return findSegmentObstacleEntry(p1, p2, unused);
}

bool LazyCoulombPlanner::nearestFreePoint(
  const PathPoint & pt, double radius, PathPoint & free_point) const
{
  unsigned int cx, cy;
  if (radius <= 0.0 || !costmap_->worldToMap(pt.x, pt.y, cx, cy)) {
    return false;
  }
  const double res = costmap_->getResolution();
  const int reach = static_cast<int>(std::ceil(radius / res));
  const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap_->getSizeInCellsY());

  double best = radius * radius;
  bool found = false;
  for (int y = std::max(0, static_cast<int>(cy) - reach);
    y <= std::min(size_y - 1, static_cast<int>(cy) + reach); ++y)
  {
    for (int x = std::max(0, static_cast<int>(cx) - reach);
      x <= std::min(size_x - 1, static_cast<int>(cx) + reach); ++x)
    {
      if (static_cast<double>(costmap_->getCost(x, y)) >= lethal_cost_threshold_) {
        continue;
      }
      double wx, wy;
      costmap_->mapToWorld(x, y, wx, wy);
      const double d2 = (wx - pt.x) * (wx - pt.x) + (wy - pt.y) * (wy - pt.y);
      if (d2 <= best) {
        best = d2;
        free_point = PathPoint(wx, wy, true);
        found = true;
      }
    }
  }
  return found;
}

bool LazyCoulombPlanner::isPathClear(const std::vector<PathPoint> & points) const
{
  for (size_t i = 0; i < points.size(); ++i) {
    if (isPointInObstacle(points[i].x, points[i].y)) {
      return false;
    }
    if (i + 1 < points.size() && segmentIntersectsObstacle(points[i], points[i + 1])) {
      return false;
    }
  }
  return true;
}

// =============================================================================
//  Repulsion physics
// =============================================================================

double LazyCoulombPlanner::coulombForceAlong(
  const PathPoint & pt, double nx, double ny) const
{
  const double res = costmap_->getResolution();
  const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap_->getSizeInCellsY());
  const unsigned char * cells = costmap_->getCharMap();

  // Point position in continuous cell coordinates (cell centres at integer + 0.5).
  const double px = (pt.x - costmap_->getOriginX()) / res;
  const double py = (pt.y - costmap_->getOriginY()) / res;

  const double radius = std::max(1.0, charge_radius_ / res);
  const double radius2 = radius * radius;
  const int reach = static_cast<int>(std::ceil(radius));
  const int cx = static_cast<int>(std::floor(px));
  const int cy = static_cast<int>(std::floor(py));

  const int x0 = std::max(0, cx - reach), x1 = std::min(size_x - 1, cx + reach);
  const int y0 = std::max(0, cy - reach), y1 = std::min(size_y - 1, cy + reach);

  double fx = 0.0, fy = 0.0;
  for (int y = y0; y <= y1; ++y) {
    const unsigned char * row = cells + static_cast<size_t>(y) * size_x;
    const double dy = py - (y + 0.5);
    for (int x = x0; x <= x1; ++x) {
      if (static_cast<double>(row[x]) < lethal_cost_threshold_) {
        continue;
      }
      const double dx = px - (x + 0.5);
      double r2 = dx * dx + dy * dy;
      if (r2 > radius2) {
        continue;
      }
      r2 = std::max(r2, 0.25);  // soften the singularity inside the charge's own cell
      // Unit vector (d / r) times magnitude 1 / r^n  ->  d / r^(n + 1)
      double w;
      if (force_exponent_ == 1.0) {
        w = 1.0 / r2;
      } else if (force_exponent_ == 2.0) {
        w = 1.0 / (r2 * std::sqrt(r2));
      } else {
        w = std::pow(r2, -0.5 * (force_exponent_ + 1.0));
      }
      fx += dx * w;
      fy += dy * w;
    }
  }
  return fx * nx + fy * ny;
}

int LazyCoulombPlanner::probeNearestClearSide(
  const PathPoint & pt, double nx, double ny) const
{
  const double res = costmap_->getResolution();
  const double max_search = std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  for (double d = res; d < max_search; d += res) {
    if (!isPointInObstacle(pt.x + nx * d, pt.y + ny * d)) {
      return 1;
    }
    if (!isPointInObstacle(pt.x - nx * d, pt.y - ny * d)) {
      return -1;
    }
  }
  return 0;
}

int LazyCoulombPlanner::preferredSide(
  const PathPoint & pt, const PathPoint & a, const PathPoint & b) const
{
  const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
  if (seg_len < 1e-9) {
    return 1;
  }
  const double nx = -(b.y - a.y) / seg_len;
  const double ny = (b.x - a.x) / seg_len;

  if (force_model_ == "probe") {
    return probeNearestClearSide(pt, nx, ny) < 0 ? -1 : 1;
  }
  const double force = coulombForceAlong(pt, nx, ny);
  // At a saddle (forces cancel) break the symmetry to the left.
  return (std::abs(force) >= force_balance_threshold_ && force < 0.0) ? -1 : 1;
}

bool LazyCoulombPlanner::pushClear(
  PathPoint & pt, const PathPoint & a, const PathPoint & b, int side, bool forced) const
{
  const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
  if (seg_len < 1e-9) {
    return false;
  }
  // Left unit normal of the segment; the waypoint only ever moves along it.
  const double nx = -(b.y - a.y) / seg_len;
  const double ny = (b.x - a.x) / seg_len;

  const double res = costmap_->getResolution();
  const double max_step = max_push_step_ > 0.0 ? max_push_step_ : res;
  const double min_step = 0.25 * res;
  const bool use_force = force_model_ == "coulomb";
  const bool keep_side = forced || commit_direction_ || !use_force;

  for (int i = 0; i < max_push_iterations_; ++i) {
    if (!isPointInObstacle(pt.x, pt.y)) {
      return true;
    }

    double step = max_step;
    if (use_force) {
      const double force = coulombForceAlong(pt, nx, ny);
      const bool balanced = std::abs(force) < force_balance_threshold_;
      const int force_side = force > 0.0 ? 1 : -1;
      if (!keep_side && !balanced) {
        side = force_side;  // pure physics: the force may reverse the waypoint
      }
      if (!balanced && force_side == side) {
        step = std::clamp(repulsion_strength_ * std::abs(force) * res, min_step, max_step);
      }
    }

    pt.x += side * nx * step;
    pt.y += side * ny * step;
  }
  return !isPointInObstacle(pt.x, pt.y);
}

// =============================================================================
//  Path utilities
// =============================================================================

std::vector<PathPoint> LazyCoulombPlanner::removeRedundant(
  const std::vector<PathPoint> & pts) const
{
  if (pts.size() < 3) {return pts;}

  std::vector<PathPoint> out;
  out.reserve(pts.size());
  out.push_back(pts.front());
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    // Keep pts[i] only if its neighbours cannot be joined directly.
    if (segmentIntersectsObstacle(out.back(), pts[i + 1])) {
      out.push_back(pts[i]);
    }
  }
  out.push_back(pts.back());
  return out;
}

std::vector<PathPoint> LazyCoulombPlanner::improveClearance(
  const std::vector<PathPoint> & pts) const
{
  if (comfort_cost_ >= lethal_cost_threshold_ || pts.size() < 2) {
    return pts;
  }

  // The path is collision free, but a straight stretch can still graze an
  // obstacle. Same idea as the main loop, applied to closeness instead of
  // collision: find where a segment gets closest (highest cost), put a waypoint
  // there and let the Coulomb force from the nearby obstacles push it away,
  // down the cost gradient. A waypoint is kept only if the path stays collision
  // free, gets no closer anywhere, and lowers the path's exposure (cost above
  // comfort_cost, summed along its length) by at least 5 %. Along a wall one
  // waypoint only lifts the middle, so the two halves are then treated in turn.
  const double res = costmap_->getResolution();
  std::vector<PathPoint> path = pts;
  int budget = 200;

  for (size_t i = 0; i + 1 < path.size() && budget > 0; ) {
    const PathPoint a = path[i];
    const PathPoint b = path[i + 1];
    const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
    PathPoint worst_point(0.0, 0.0);
    const double worst = segmentMaxCost(a, b, worst_point);
    if (seg_len < 4.0 * res || worst <= comfort_cost_) {
      ++i;
      continue;
    }

    const double nx = -(b.y - a.y) / seg_len;
    const double ny = (b.x - a.x) / seg_len;
    const double force = coulombForceAlong(worst_point, nx, ny);
    bool improved = false;
    if (std::abs(force) >= force_balance_threshold_) {
      const double side = force > 0.0 ? 1.0 : -1.0;
      PathPoint q = worst_point;
      double cost = costAt(q.x, q.y);
      bool moved = false;
      // Keep the detour gentle: a large sideways offset on a short stretch would
      // make the path double back on itself.
      const double reach = std::min(
        max_clearance_push_,
        kMaxOffsetRatio * std::min(
          std::hypot(worst_point.x - a.x, worst_point.y - a.y),
          std::hypot(worst_point.x - b.x, worst_point.y - b.y)));
      PathPoint probe = q;
      int flat_steps = 0;
      for (double d = res; d <= reach + 1e-9 && cost > comfort_cost_; d += res) {
        probe.x += side * nx * res;
        probe.y += side * ny * res;
        const double next_cost = costAt(probe.x, probe.y);
        if (next_cost > cost) {
          break;
        }
        if (next_cost == cost) {
          // A flat step (same cell again, or equal costs) is not the bottom yet.
          if (++flat_steps > kMaxFlatSteps) {
            break;
          }
          continue;
        }
        q = probe;
        cost = next_cost;
        flat_steps = 0;
        moved = true;
      }
      PathPoint unused(0.0, 0.0);
      if (moved &&
        segmentMaxCost(a, q, unused) <= worst && segmentMaxCost(q, b, unused) <= worst &&
        segmentExposure(a, q) + segmentExposure(q, b) < 0.95 * segmentExposure(a, b))
      {
        q.is_locked = true;
        path.insert(path.begin() + i + 1, q);
        improved = true;
        --budget;
      }
    }
    if (!improved) {
      ++i;  // nothing better here; otherwise re-examine (a -> q) next
    }
  }
  return path;
}

std::vector<PathPoint> LazyCoulombPlanner::chaikinSmooth(
  const std::vector<PathPoint> & pts) const
{
  if (pts.size() < 3) {return pts;}

  // Chaikin replaces each corner P by two points, Q on the incoming segment and
  // R on the outgoing one, a quarter of the way from P. Q and R lie on segments
  // already known to be clear, so the only new piece is the chord Q -> R. If the
  // chord touches an obstacle, or comes closer to one than allowed, the cut is
  // retried closer to P, and if no cut fits the corner is kept. The smoothed
  // path is therefore collision free by construction.
  static const double kCuts[] = {0.25, 0.125, 0.0625};

  std::vector<PathPoint> out;
  out.reserve(pts.size() * 2);
  out.push_back(pts.front());

  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    const PathPoint & prev = pts[i - 1];
    const PathPoint & corner = pts[i];
    const PathPoint & next = pts[i + 1];

    // Rounding a corner moves the path towards the obstacle it goes around. A
    // cut may give back part of the clearance (smoothing_cost_slack), but never
    // reach an obstacle cell.
    const double allowed = std::min(
      lethal_cost_threshold_ - 1.0,
      std::max(comfort_cost_, costAt(corner.x, corner.y)) + smoothing_cost_slack_);
    bool cut = false;
    for (const double f : kCuts) {
      const PathPoint q(corner.x + f * (prev.x - corner.x), corner.y + f * (prev.y - corner.y));
      const PathPoint r(corner.x + f * (next.x - corner.x), corner.y + f * (next.y - corner.y));
      PathPoint unused(0.0, 0.0);
      if (segmentMaxCost(q, r, unused) <= allowed) {
        out.push_back(q);
        out.push_back(r);
        cut = true;
        break;
      }
    }
    if (!cut) {
      out.push_back(corner);
    }
  }

  out.push_back(pts.back());
  return out;
}

std::vector<PathPoint> LazyCoulombPlanner::densify(
  const std::vector<PathPoint> & pts, double spacing) const
{
  std::vector<PathPoint> out;
  if (pts.empty()) {return out;}
  out.push_back(pts.front());
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double dx = pts[i + 1].x - pts[i].x;
    const double dy = pts[i + 1].y - pts[i].y;
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / spacing)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      out.emplace_back(pts[i].x + dx * t, pts[i].y + dy * t);
    }
  }
  return out;
}

nav_msgs::msg::Path LazyCoulombPlanner::toNavPath(
  const std::vector<PathPoint> & points,
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Quaternion & goal_orientation) const
{
  nav_msgs::msg::Path nav_path;
  nav_path.header = header;
  nav_path.poses.reserve(points.size());

  double yaw = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    pose.pose.position.z = 0.0;
    if (i + 1 < points.size()) {
      const double dx = points[i + 1].x - points[i].x;
      const double dy = points[i + 1].y - points[i].y;
      if (std::hypot(dx, dy) > 1e-9) {
        yaw = std::atan2(dy, dx);
      }
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      pose.pose.orientation = goal_orientation;
    }
    nav_path.poses.push_back(pose);
  }

  return nav_path;
}

}  // namespace lazy_coulomb_planner
