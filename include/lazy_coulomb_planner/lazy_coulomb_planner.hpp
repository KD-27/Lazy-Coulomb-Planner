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
 * Implements a reactive path planning algorithm inspired by electrostatic
 * repulsion (Coulomb's law). Starting from the laziest assumption - a
 * straight line from start to goal - it iteratively detects obstacle
 * intersections and pushes waypoints clear using perpendicular repulsion
 * forces, then locks them in place.
 *
 * Algorithm steps:
 *   1. Initialize path as straight line S -> G
 *   2. Find first segment that crosses an obstacle (via costmap)
 *   3. Insert a new point at the intersection
 *   4. Push it perpendicularly until outside all obstacles
 *   5. Lock it and continue scanning from the start
 *   6. Repeat until no segment intersects any obstacle
 *   7. Apply optional Chaikin smoothing to final path
 */
class LazyCoulombPlanner : public nav2_core::GlobalPlanner
{
public:
  LazyCoulombPlanner() = default;
  ~LazyCoulombPlanner() = default;

  // Nav2 lifecycle interface

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
   * @return nav_msgs::msg::Path with waypoints from start to goal
   * @throws nav2_core::PlannerException if no valid path found
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Coordinate helpers

  /**
   * @brief Convert world (x,y) to costmap cell (mx, my).
   * @return true if conversion succeeded (point is within map bounds)
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  /** @brief Convert costmap cell (mx, my) to world (x, y). */
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  // Obstacle queries

  /**
   * @brief Check if a world-frame point is inside an obstacle or inflation zone.
   *        Uses lethal + inscribed cost threshold from the costmap.
   */
  bool isPointInObstacle(double wx, double wy) const;

  /**
   * @brief Check if any point along segment (p1 -> p2) hits an obstacle.
   *        Samples the segment at sub-cell resolution.
   */
  bool segmentIntersectsObstacle(const PathPoint & p1, const PathPoint & p2) const;

  /**
   * @brief Find the first world-frame point along (p1 -> p2) that enters an obstacle.
   * @return true if an intersection was found; result written to intersection
   */
  bool findSegmentObstacleEntry(
    const PathPoint & p1,
    const PathPoint & p2,
    PathPoint & intersection) const;

  // Repulsion physics

  /**
   * @brief Compute the perpendicular-to-path repulsion force at a given point.
   *
   * Samples left and right of the path direction in the costmap to find which
   * side has clear space closer. Falls back to pushing directly away from the
   * nearest high-cost cell if neither perpendicular direction is clear.
   *
   * @param point Point currently inside an obstacle
   * @param path_ref_prev Reference point before (for path direction)
   * @param path_ref_next Reference point after (for path direction)
   * @param force_x Output: x component of repulsion force
   * @param force_y Output: y component of repulsion force
   */
  void calculateRepulsionForce(
    const PathPoint & point,
    const PathPoint & path_ref_prev,
    const PathPoint & path_ref_next,
    double & force_x,
    double & force_y) const;

  /**
   * @brief Find the nearest obstacle cell and return a push-away direction.
   *        Used as deterministic fallback when perpendicular push is blocked.
   */
  void pushAwayFromNearestObstacle(
    const PathPoint & point,
    double & force_x,
    double & force_y) const;

  // Path utilities

  /** @brief Build the initial straight-line path from start to goal. */
  std::vector<PathPoint> initializePath(
    const PathPoint & start,
    const PathPoint & goal) const;

  /**
   * @brief Apply one round of Chaikin corner-cutting to smooth the path.
   *        Keeps start and goal fixed.
   */
  std::vector<PathPoint> chaikinSmooth(const std::vector<PathPoint> & points) const;

  /** @brief Convert internal PathPoint vector to nav_msgs::msg::Path. */
  nav_msgs::msg::Path toNavPath(
    const std::vector<PathPoint> & points,
    const std_msgs::msg::Header & header) const;

  // Members
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("LazyCoulombPlanner")};

  // Tunable parameters (set via nav2_params.yaml)
  int max_iterations_;  // Hard cap on total algorithm iterations
  int max_push_iterations_;  // Max steps to push a single point clear
  double step_size_;  // Integration step size when pushing (meters)
  double repulsion_strength_;  // Base magnitude of repulsion force
  double force_balance_threshold_;  // Below this force magnitude, use fallback
  double perturbation_strength_;  // Deterministic fallback nudge strength
  int initial_path_points_;  // Number of points in initial straight line
  bool enable_smoothing_;  // Apply Chaikin smoothing to final path
  int smoothing_iterations_;  // Number of Chaikin passes
  double lethal_cost_threshold_;  // Costmap cost value considered obstacle
  int segment_check_steps_;  // Samples per unit length for segment checks
};

}  // namespace lazy_coulomb_planner

#endif  // LAZY_COULOMB_PLANNER__LAZY_COULOMB_PLANNER_HPP_
