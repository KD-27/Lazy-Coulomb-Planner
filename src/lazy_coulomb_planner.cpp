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

  max_iterations_ = declare("max_iterations", rclcpp::ParameterValue(500)).as_int();
  max_push_iterations_ = declare("max_push_iterations", rclcpp::ParameterValue(400)).as_int();
  force_model_ = declare("force_model", rclcpp::ParameterValue("coulomb")).as_string();
  force_exponent_ = declare("force_exponent", rclcpp::ParameterValue(2.0)).as_double();
  charge_radius_ = declare("charge_radius", rclcpp::ParameterValue(1.0)).as_double();
  repulsion_strength_ = declare("repulsion_strength", rclcpp::ParameterValue(1.0)).as_double();
  max_push_step_ = declare("max_push_step", rclcpp::ParameterValue(0.0)).as_double();
  force_balance_threshold_ =
    declare("force_balance_threshold", rclcpp::ParameterValue(0.05)).as_double();
  commit_direction_ = declare("commit_direction", rclcpp::ParameterValue(true)).as_bool();
  simplify_path_ = declare("simplify_path", rclcpp::ParameterValue(true)).as_bool();
  enable_smoothing_ = declare("enable_smoothing", rclcpp::ParameterValue(true)).as_bool();
  smoothing_iterations_ = declare("smoothing_iterations", rclcpp::ParameterValue(3)).as_int();
  output_spacing_ = declare("output_spacing", rclcpp::ParameterValue(0.1)).as_double();
  lethal_cost_threshold_ =
    declare("lethal_cost_threshold", rclcpp::ParameterValue(253.0)).as_double();

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
  if (isPointInObstacle(start.x, start.y)) {
    throw nav2_core::StartOccupied("LazyCoulombPlanner: Start is inside an obstacle");
  }
  if (isPointInObstacle(goal.x, goal.y)) {
    throw nav2_core::GoalOccupied("LazyCoulombPlanner: Goal is inside an obstacle");
  }

  // Lazy start: the straight line. Segments before `seg` are verified clear.
  std::vector<PathPoint> path{start, goal};
  int corrections = 0;
  size_t seg = 0;

  while (seg + 1 < path.size()) {
    PathPoint waypoint(0.0, 0.0);
    if (!findSegmentObstacleEntry(path[seg], path[seg + 1], waypoint)) {
      ++seg;
      continue;
    }

    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("LazyCoulombPlanner: Planning was cancelled");
    }
    if (++corrections > max_iterations_) {
      throw nav2_core::NoValidPathCouldBeFound(
              "LazyCoulombPlanner: Correction budget (" + std::to_string(max_iterations_) +
              ") exhausted");
    }
    if (!pushClear(waypoint, path[seg], path[seg + 1])) {
      throw nav2_core::NoValidPathCouldBeFound(
              "LazyCoulombPlanner: A waypoint could not be pushed clear of obstacles");
    }

    waypoint.is_locked = true;
    path.insert(path.begin() + seg + 1, waypoint);
    // Stay on `seg`: the new segment (path[seg] -> waypoint) is checked next.
  }

  if (simplify_path_) {
    path = removeRedundant(path);
  }

  if (enable_smoothing_ && path.size() >= 3) {
    // Chaikin cuts corners, so each pass is kept only if it is still collision free.
    for (int i = 0; i < smoothing_iterations_; ++i) {
      std::vector<PathPoint> smoothed = chaikinSmooth(path);
      if (!isPathClear(smoothed)) {
        break;
      }
      path = std::move(smoothed);
    }
  }

  if (output_spacing_ > 0.0) {
    path = densify(path, output_spacing_);
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

bool LazyCoulombPlanner::findSegmentObstacleEntry(
  const PathPoint & p1, const PathPoint & p2, PathPoint & intersection) const
{
  // Exact grid traversal (Amanatides & Woo): visits every cell the segment
  // touches, so a corner clipped by a short chord cannot be skipped.
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
  // Segment parameter t in [0, 1] at which the next vertical / horizontal cell edge is crossed.
  const double dt_x = dx != 0.0 ? std::abs(1.0 / dx) : inf;
  const double dt_y = dy != 0.0 ? std::abs(1.0 / dy) : inf;
  double t_next_x = dx != 0.0 ? ((dx > 0.0 ? cx + 1 - x0 : x0 - cx) * dt_x) : inf;
  double t_next_y = dy != 0.0 ? ((dy > 0.0 ? cy + 1 - y0 : y0 - cy) * dt_y) : inf;

  double t_enter = 0.0;
  const int max_cells = std::abs(end_x - cx) + std::abs(end_y - cy) + 1;
  for (int n = 0; n < max_cells; ++n) {
    const double t_exit = std::min({t_next_x, t_next_y, 1.0});
    const bool outside = cx < 0 || cy < 0 || cx >= size_x || cy >= size_y;
    if (outside ||
      static_cast<double>(cells[static_cast<size_t>(cy) * size_x + cx]) >= lethal_cost_threshold_)
    {
      // Middle of the chord through the offending cell: guaranteed to lie inside it.
      const double t = 0.5 * (t_enter + t_exit);
      intersection = PathPoint(p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t);
      return true;
    }
    if (t_exit >= 1.0) {
      break;
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
  return false;
}

bool LazyCoulombPlanner::segmentIntersectsObstacle(
  const PathPoint & p1, const PathPoint & p2) const
{
  PathPoint unused(0.0, 0.0);
  return findSegmentObstacleEntry(p1, p2, unused);
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

bool LazyCoulombPlanner::pushClear(
  PathPoint & pt, const PathPoint & a, const PathPoint & b) const
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

  int side = 0;  // +1 left, -1 right, 0 undecided
  if (force_model_ == "probe") {
    side = probeNearestClearSide(pt, nx, ny);
    if (side == 0) {
      return false;
    }
  }

  for (int i = 0; i < max_push_iterations_; ++i) {
    if (!isPointInObstacle(pt.x, pt.y)) {
      return true;
    }

    double step = max_step;
    if (force_model_ == "coulomb") {
      const double force = coulombForceAlong(pt, nx, ny);
      const bool balanced = std::abs(force) < force_balance_threshold_;
      const int force_side = force > 0.0 ? 1 : -1;

      if (side == 0 || !commit_direction_) {
        // Physics picks the side. At a saddle keep the previous side, else kick left.
        if (!balanced) {
          side = force_side;
        } else if (side == 0) {
          side = 1;
        }
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

std::vector<PathPoint> LazyCoulombPlanner::chaikinSmooth(
  const std::vector<PathPoint> & pts) const
{
  if (pts.size() < 3) {return pts;}

  std::vector<PathPoint> out;
  out.reserve(pts.size() * 2);
  out.push_back(pts.front());

  for (size_t i = 0; i < pts.size() - 1; ++i) {
    const PathPoint & p0 = pts[i];
    const PathPoint & p1 = pts[i + 1];
    PathPoint q(p0.x * 0.75 + p1.x * 0.25, p0.y * 0.75 + p1.y * 0.25);
    PathPoint r(p0.x * 0.25 + p1.x * 0.75, p0.y * 0.25 + p1.y * 0.75);
    if (i > 0) {out.push_back(q);}
    if (i < pts.size() - 2) {out.push_back(r);}
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
