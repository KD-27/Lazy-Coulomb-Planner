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

#ifndef LAZY_COULOMB_PLANNER__LAZY_COULOMB_PLANNER_HPP_
#define LAZY_COULOMB_PLANNER__LAZY_COULOMB_PLANNER_HPP_

#include <functional>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"

namespace lazy_coulomb_planner
{

/**
 * @brief A path point used internally during planning.
 *        Stores world coordinates plus algorithm state flags.
 */
struct PathPoint
{
  double x;  // world frame x (meters)
  double y;  // world frame y (meters)
  bool is_locked;  // locked points won't be moved again
  bool is_active;  // currently being pushed by repulsion

  PathPoint(double x_, double y_, bool locked = false)
  : x(x_), y(y_), is_locked(locked), is_active(false) {}
};

/**
 * @brief Lazy Coulomb Planner - Nav2 Global Planner Plugin
 *
 * Starts from the laziest assumption - a straight line from start to goal -
 * and only does work where that line crosses an obstacle. Each crossing gets a
 * waypoint, which is pushed sideways (perpendicular to the segment it sits on)
 * until it is clear, then locked.
 *
 * The push is driven by a summed Coulomb force: every obstacle cell within
 * `charge_radius` of the waypoint repels it with magnitude 1 / r^force_exponent
 * (r in cells). The component of that force perpendicular to the path decides
 * which side the waypoint leaves on and how fast it moves.
 *
 * The planner never returns a path it knows to be in collision: if a waypoint
 * cannot be cleared, or the correction budget runs out, it throws
 * nav2_core::NoValidPathCouldBeFound so a fallback planner can take over.
 */
class LazyCoulombPlanner : public nav2_core::GlobalPlanner
{
public:
  LazyCoulombPlanner() = default;
  ~LazyCoulombPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief Core planning method called by Nav2 when a new goal is received.
   * @param start Current robot pose in map frame
   * @param goal Target pose in map frame
   * @param cancel_checker Returns true if the planning request was cancelled
   * @return nav_msgs::msg::Path with waypoints from start to goal
   * @throws nav2_core::PlannerException (or a subclass) if no valid path found
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  /**
   * @brief Testing utility: inject a raw Costmap2D, bypassing Costmap2DROS.
   *        Call after configure() and before createPlan(). Not for production.
   */
  void setTestCostmap(nav2_costmap_2d::Costmap2D * cm)
  {
    costmap_ = cm;
  }

protected:
  /** @brief True if the world point is outside the map or on an obstacle cell. */
  bool isPointInObstacle(double wx, double wy) const;

  /** @brief True if any cell touched by the segment (p1 -> p2) is an obstacle. */
  bool segmentIntersectsObstacle(const PathPoint & p1, const PathPoint & p2) const;

  /** @brief A point inside the first obstacle cell touched by (p1 -> p2), if any. */
  bool findSegmentObstacleEntry(
    const PathPoint & p1, const PathPoint & p2, PathPoint & intersection) const;

  /** @brief True if every segment of the polyline is collision free. */
  bool isPathClear(const std::vector<PathPoint> & points) const;

  /**
   * @brief Push a waypoint sideways, perpendicular to (a -> b), until it is clear.
   * @return false if it could not be cleared within max_push_iterations
   */
  bool pushClear(PathPoint & pt, const PathPoint & a, const PathPoint & b) const;

  /**
   * @brief Summed Coulomb force on a point from all obstacle cells within
   *        charge_radius, projected onto the unit vector (nx, ny).
   *        Distances are in cells; magnitude per cell is 1 / r^force_exponent.
   */
  double coulombForceAlong(const PathPoint & pt, double nx, double ny) const;

  /**
   * @brief Legacy direction rule: +1 (left) or -1 (right), whichever side
   *        reaches free space sooner along the normal; 0 if neither does.
   */
  int probeNearestClearSide(const PathPoint & pt, double nx, double ny) const;

  /** @brief Drop waypoints whose neighbours can be joined directly. */
  std::vector<PathPoint> removeRedundant(const std::vector<PathPoint> & points) const;

  /** @brief One round of Chaikin corner-cutting. Keeps start and goal fixed. */
  std::vector<PathPoint> chaikinSmooth(const std::vector<PathPoint> & points) const;

  /** @brief Resample the polyline so consecutive poses are at most `spacing` apart. */
  std::vector<PathPoint> densify(const std::vector<PathPoint> & points, double spacing) const;

  /** @brief Convert to nav_msgs::msg::Path with headings along the path. */
  nav_msgs::msg::Path toNavPath(
    const std::vector<PathPoint> & points,
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Quaternion & goal_orientation) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("LazyCoulombPlanner")};

  // Tunable parameters
  int max_iterations_;              // Max waypoints inserted before giving up
  int max_push_iterations_;         // Max steps to push a single waypoint clear
  std::string force_model_;         // "coulomb" or "probe" (legacy nearest-clear-side rule)
  double force_exponent_;           // n in 1 / r^n  (1.0 or 2.0)
  double charge_radius_;            // Obstacle cells within this range repel (m)
  double repulsion_strength_;       // Cells moved per unit of normalised force
  double max_push_step_;            // Cap on a single push step (m); <= 0 means one cell
  double force_balance_threshold_;  // Below this |force| the push is a fixed kick
  bool commit_direction_;           // Keep the side chosen on the first step
  bool simplify_path_;              // Remove redundant waypoints after solving
  bool enable_smoothing_;           // Chaikin smoothing, kept only if collision free
  int smoothing_iterations_;        // Number of Chaikin passes
  double output_spacing_;           // Max distance between output poses (m); <= 0 disables
  double lethal_cost_threshold_;    // Cost at or above this is an obstacle
};

}  // namespace lazy_coulomb_planner

#endif  // LAZY_COULOMB_PLANNER__LAZY_COULOMB_PLANNER_HPP_
