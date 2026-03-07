#include "lazy_coulomb_planner/lazy_coulomb_planner.hpp"

#include <algorithm>
#include <stdexcept>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

// ── Register as a Nav2 plugin ─────────────────────────────────────────────────
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
  node_        = parent;
  name_        = name;
  tf_          = tf;
  costmap_ros_ = costmap_ros;
  costmap_     = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LazyCoulombPlanner: Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();

  // ── Declare and read all parameters ────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations",          rclcpp::ParameterValue(1000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_push_iterations",     rclcpp::ParameterValue(300));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size",               rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".repulsion_strength",      rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".force_balance_threshold", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perturbation_strength",   rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".initial_path_points",     rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_smoothing",        rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoothing_iterations",    rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost_threshold",   rclcpp::ParameterValue(253.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".segment_check_steps",     rclcpp::ParameterValue(20));

  max_iterations_          = node->get_parameter(name_ + ".max_iterations").as_int();
  max_push_iterations_     = node->get_parameter(name_ + ".max_push_iterations").as_int();
  step_size_               = node->get_parameter(name_ + ".step_size").as_double();
  repulsion_strength_      = node->get_parameter(name_ + ".repulsion_strength").as_double();
  force_balance_threshold_ = node->get_parameter(name_ + ".force_balance_threshold").as_double();
  perturbation_strength_   = node->get_parameter(name_ + ".perturbation_strength").as_double();
  initial_path_points_     = node->get_parameter(name_ + ".initial_path_points").as_int();
  enable_smoothing_        = node->get_parameter(name_ + ".enable_smoothing").as_bool();
  smoothing_iterations_    = node->get_parameter(name_ + ".smoothing_iterations").as_int();
  lethal_cost_threshold_   = node->get_parameter(name_ + ".lethal_cost_threshold").as_double();
  segment_check_steps_     = node->get_parameter(name_ + ".segment_check_steps").as_int();

  RCLCPP_INFO(logger_,
    "LazyCoulombPlanner configured: max_iter=%d, step=%.3f, repulsion=%.3f",
    max_iterations_, step_size_, repulsion_strength_);
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
//  createPlan  –  the heart of the plugin
// =============================================================================

nav_msgs::msg::Path LazyCoulombPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  // ── 1. Validate inputs ──────────────────────────────────────────────────────
  if (!costmap_) {
    throw nav2_core::PlannerException("LazyCoulombPlanner: Costmap not available");
  }

  PathPoint start(start_pose.pose.position.x, start_pose.pose.position.y, true);
  PathPoint goal (goal_pose.pose.position.x,  goal_pose.pose.position.y,  true);

  // Sanity: verify start and goal are within the costmap
  {
    unsigned int mx, my;
    if (!worldToMap(start.x, start.y, mx, my)) {
      throw nav2_core::PlannerException("LazyCoulombPlanner: Start is outside costmap");
    }
    if (!worldToMap(goal.x, goal.y, mx, my)) {
      throw nav2_core::PlannerException("LazyCoulombPlanner: Goal is outside costmap");
    }
  }

  RCLCPP_DEBUG(logger_,
    "Planning from (%.2f, %.2f) to (%.2f, %.2f)",
    start.x, start.y, goal.x, goal.y);

  // ── 2. Initialize straight-line path ───────────────────────────────────────
  std::vector<PathPoint> path = initializePath(start, goal);

  // Track which indices are "locked" (won't be moved again).
  // Both endpoints are always locked.
  std::vector<bool> locked(path.size(), false);
  locked.front() = true;
  locked.back()  = true;

  int total_iterations = 0;

  // ── 3. Main loop: find & resolve intersections ──────────────────────────────
  while (total_iterations < max_iterations_) {

    // ── 3a. Find the FIRST segment (from start) that crosses an obstacle ──────
    int intersecting_seg = -1;
    PathPoint intersection_pt(0, 0);

    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
      if (findSegmentObstacleEntry(path[i], path[i + 1], intersection_pt)) {
        intersecting_seg = i;
        break;
      }
    }

    // ── 3b. No more intersections → path is clear ─────────────────────────────
    if (intersecting_seg < 0) {
      RCLCPP_DEBUG(logger_,
        "LCP solved in %d iterations, path has %zu points",
        total_iterations, path.size());
      break;
    }

    // ── 3c. Insert a new waypoint at the intersection point ───────────────────
    int insert_idx = intersecting_seg + 1;
    intersection_pt.is_locked = false;
    intersection_pt.is_active = true;

    path.insert(path.begin() + insert_idx, intersection_pt);
    locked.insert(locked.begin() + insert_idx, false);

    total_iterations++;

    // ── 3d. Find the nearest locked anchor points before & after ─────────────
    //        to establish the reference path direction for repulsion
    int prev_locked = 0;
    int next_locked = static_cast<int>(path.size()) - 1;

    for (int i = insert_idx - 1; i >= 0; --i) {
      if (locked[i]) { prev_locked = i; break; }
    }
    for (int i = insert_idx + 1; i < static_cast<int>(path.size()); ++i) {
      if (locked[i]) { next_locked = i; break; }
    }

    // ── 3e. Push the inserted point until it leaves the obstacle ──────────────
    int push_iters = 0;

    while (push_iters < max_push_iterations_ && total_iterations < max_iterations_) {
      PathPoint & pt = path[insert_idx];

      if (!isPointInObstacle(pt.x, pt.y)) {
        break;  // Successfully pushed clear
      }

      double fx, fy;
      calculateRepulsionForce(pt, path[prev_locked], path[next_locked], fx, fy);

      // Deterministic fallback: if forces are near zero (symmetric obstacle),
      // nudge diagonally toward the perpendicular-left direction.
      double mag = std::hypot(fx, fy);
      if (mag < force_balance_threshold_) {
        // Compute path direction and take perpendicular-left
        double pdx = path[next_locked].x - path[prev_locked].x;
        double pdy = path[next_locked].y - path[prev_locked].y;
        double plen = std::hypot(pdx, pdy);
        if (plen > 1e-6) {
          pdx /= plen; pdy /= plen;
        }
        // Perpendicular left
        fx = (-pdy) * perturbation_strength_;
        fy = ( pdx) * perturbation_strength_;
      } else {
        fx = (fx / mag) * repulsion_strength_;
        fy = (fy / mag) * repulsion_strength_;
      }

      // Integrate one step
      pt.x += fx * step_size_;
      pt.y += fy * step_size_;

      // Clamp to costmap bounds
      double ox = costmap_->getOriginX();
      double oy = costmap_->getOriginY();
      double w  = costmap_->getSizeInMetersX();
      double h  = costmap_->getSizeInMetersY();
      pt.x = std::clamp(pt.x, ox, ox + w);
      pt.y = std::clamp(pt.y, oy, oy + h);

      push_iters++;
      total_iterations++;
    }

    // ── 3f. Lock the point regardless (even if still inside – rare edge case) ─
    locked[insert_idx] = true;
    path[insert_idx].is_locked = true;
    path[insert_idx].is_active = false;

    // ── 3g. Warn if we couldn't push clear ───────────────────────────────────
    if (isPointInObstacle(path[insert_idx].x, path[insert_idx].y)) {
      RCLCPP_WARN(logger_,
        "LCP: Point %d could not be pushed fully clear after %d push iterations",
        insert_idx, push_iters);
    }

    // ── 3h. Remove any unlocked intermediate points (keep path minimal) ───────
    std::vector<PathPoint> cleaned;
    std::vector<bool>      cleaned_locked;
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
      if (locked[i]) {
        cleaned.push_back(path[i]);
        cleaned_locked.push_back(true);
      }
    }
    path   = cleaned;
    locked = cleaned_locked;
  }

  // ── 4. Check iteration budget ──────────────────────────────────────────────
  if (total_iterations >= max_iterations_) {
    RCLCPP_WARN(logger_,
      "LCP: Reached max iterations (%d). Path may clip obstacles.", max_iterations_);
  }

  // ── 5. Final validation: ensure at least start & goal are present ──────────
  if (path.empty()) {
    throw nav2_core::PlannerException("LazyCoulombPlanner: Path is empty after planning");
  }

  // ── 6. Optional Chaikin smoothing ─────────────────────────────────────────
  if (enable_smoothing_ && path.size() >= 3) {
    for (int i = 0; i < smoothing_iterations_; ++i) {
      path = chaikinSmooth(path);
    }
  }

  // ── 7. Pack into nav_msgs::msg::Path and return ────────────────────────────
  std_msgs::msg::Header header;
  header.stamp    = rclcpp::Clock().now();
  header.frame_id = global_frame_;

  nav_msgs::msg::Path nav_path = toNavPath(path, header);

  RCLCPP_INFO(logger_,
    "LCP: Plan created with %zu poses in %d iterations",
    nav_path.poses.size(), total_iterations);

  return nav_path;
}

// =============================================================================
//  Coordinate helpers
// =============================================================================

bool LazyCoulombPlanner::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

void LazyCoulombPlanner::mapToWorld(
  unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

// =============================================================================
//  Obstacle queries
// =============================================================================

bool LazyCoulombPlanner::isPointInObstacle(double wx, double wy) const
{
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    // Outside map bounds → treat as obstacle
    return true;
  }
  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost) >= lethal_cost_threshold_;
}

bool LazyCoulombPlanner::segmentIntersectsObstacle(
  const PathPoint & p1, const PathPoint & p2) const
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double len = std::hypot(dx, dy);

  int steps = std::max(
    segment_check_steps_,
    static_cast<int>(std::ceil(len / costmap_->getResolution())));

  for (int i = 1; i < steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double x = p1.x + dx * t;
    double y = p1.y + dy * t;
    if (isPointInObstacle(x, y)) return true;
  }
  return false;
}

bool LazyCoulombPlanner::findSegmentObstacleEntry(
  const PathPoint & p1, const PathPoint & p2, PathPoint & intersection) const
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double len = std::hypot(dx, dy);

  int steps = std::max(
    segment_check_steps_ * 2,
    static_cast<int>(std::ceil(len / costmap_->getResolution())));

  for (int i = 1; i < steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double x = p1.x + dx * t;
    double y = p1.y + dy * t;
    if (isPointInObstacle(x, y)) {
      intersection = PathPoint(x, y);
      return true;
    }
  }
  return false;
}

// =============================================================================
//  Repulsion physics
// =============================================================================

void LazyCoulombPlanner::calculateRepulsionForce(
  const PathPoint & point,
  const PathPoint & path_ref_prev,
  const PathPoint & path_ref_next,
  double & force_x,
  double & force_y) const
{
  // ── Path direction (prev locked → next locked) ────────────────────────────
  double path_dx = path_ref_next.x - path_ref_prev.x;
  double path_dy = path_ref_next.y - path_ref_prev.y;
  double path_len = std::hypot(path_dx, path_dy);

  if (path_len < 1e-6) {
    // Degenerate: start == goal
    pushAwayFromNearestObstacle(point, force_x, force_y);
    return;
  }

  path_dx /= path_len;
  path_dy /= path_len;

  // Perpendicular directions (left and right of path)
  double left_x  = -path_dy,  left_y  =  path_dx;
  double right_x =  path_dy,  right_y = -path_dx;

  // ── Find clearance in each perpendicular direction ────────────────────────
  double res  = costmap_->getResolution();
  double max_search = std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());

  double dist_left_clear  = -1.0;
  double dist_right_clear = -1.0;

  for (double d = res; d < max_search; d += res) {
    if (dist_left_clear < 0 &&
        !isPointInObstacle(point.x + left_x * d, point.y + left_y * d))
    {
      dist_left_clear = d;
    }
    if (dist_right_clear < 0 &&
        !isPointInObstacle(point.x + right_x * d, point.y + right_y * d))
    {
      dist_right_clear = d;
    }
    if (dist_left_clear > 0 && dist_right_clear > 0) break;
  }

  // ── Choose the direction with shorter distance to clear space ─────────────
  if (dist_left_clear > 0 && (dist_right_clear < 0 || dist_left_clear <= dist_right_clear)) {
    force_x = left_x;
    force_y = left_y;
  } else if (dist_right_clear > 0) {
    force_x = right_x;
    force_y = right_y;
  } else {
    // Neither perpendicular direction is clear → push away from nearest obstacle
    pushAwayFromNearestObstacle(point, force_x, force_y);
  }
}

void LazyCoulombPlanner::pushAwayFromNearestObstacle(
  const PathPoint & point,
  double & force_x,
  double & force_y) const
{
  // Search an expanding window around the current cell for the nearest obstacle,
  // then push in the opposite direction.
  unsigned int cx, cy;
  if (!worldToMap(point.x, point.y, cx, cy)) {
    force_x = 0; force_y = 0;
    return;
  }

  // Find nearest non-obstacle cell within search radius
  int search_radius = 20;  // cells
  double best_dist = std::numeric_limits<double>::max();
  int best_dx = 0, best_dy = 0;

  int map_w = static_cast<int>(costmap_->getSizeInCellsX());
  int map_h = static_cast<int>(costmap_->getSizeInCellsY());

  for (int dy = -search_radius; dy <= search_radius; ++dy) {
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
      int nx = static_cast<int>(cx) + dx;
      int ny = static_cast<int>(cy) + dy;
      if (nx < 0 || ny < 0 || nx >= map_w || ny >= map_h) continue;

      unsigned char cost = costmap_->getCost(
        static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));

      if (static_cast<double>(cost) < lethal_cost_threshold_) {
        double d = std::hypot(dx, dy);
        if (d < best_dist) {
          best_dist = d;
          best_dx = -dx;  // push AWAY from obstacle
          best_dy = -dy;
        }
      }
    }
  }

  double mag = std::hypot(best_dx, best_dy);
  if (mag > 1e-6) {
    force_x = best_dx / mag;
    force_y = best_dy / mag;
  } else {
    // Completely surrounded – push toward goal as last resort
    force_x = 0.0;
    force_y = 1.0;
  }
}

// =============================================================================
//  Path utilities
// =============================================================================

std::vector<PathPoint> LazyCoulombPlanner::initializePath(
  const PathPoint & start, const PathPoint & goal) const
{
  std::vector<PathPoint> path;
  path.reserve(initial_path_points_ + 1);

  for (int i = 0; i <= initial_path_points_; ++i) {
    double t = static_cast<double>(i) / initial_path_points_;
    PathPoint p(
      start.x + (goal.x - start.x) * t,
      start.y + (goal.y - start.y) * t);
    p.is_locked = (i == 0 || i == initial_path_points_);
    path.push_back(p);
  }

  return path;
}

std::vector<PathPoint> LazyCoulombPlanner::chaikinSmooth(
  const std::vector<PathPoint> & pts) const
{
  if (pts.size() < 3) return pts;

  std::vector<PathPoint> out;
  out.reserve(pts.size() * 2);

  // Always keep start
  out.push_back(pts.front());

  for (size_t i = 0; i < pts.size() - 1; ++i) {
    const PathPoint & p0 = pts[i];
    const PathPoint & p1 = pts[i + 1];

    // Q = 75% p0 + 25% p1
    PathPoint q(p0.x * 0.75 + p1.x * 0.25,
                p0.y * 0.75 + p1.y * 0.25);

    // R = 25% p0 + 75% p1
    PathPoint r(p0.x * 0.25 + p1.x * 0.75,
                p0.y * 0.25 + p1.y * 0.75);

    // Skip Q for the very first segment (we already have start)
    if (i > 0) out.push_back(q);
    if (i < pts.size() - 2) out.push_back(r);
  }

  // Always keep goal
  out.push_back(pts.back());

  return out;
}

nav_msgs::msg::Path LazyCoulombPlanner::toNavPath(
  const std::vector<PathPoint> & points,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path nav_path;
  nav_path.header = header;
  nav_path.poses.reserve(points.size());

  for (const auto & pt : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;  // identity rotation
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    nav_path.poses.push_back(pose);
  }

  return nav_path;
}

}  // namespace lazy_coulomb_planner
