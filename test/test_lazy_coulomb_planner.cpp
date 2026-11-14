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

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "lazy_coulomb_planner/lazy_coulomb_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using lazy_coulomb_planner::LazyCoulombPlanner;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace
{

const double kRes = 0.05;
const unsigned int kCells = 200;  // 10 m x 10 m map with its origin at (0, 0)

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

bool never() {return false;}

class LcpTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lcp_test");
    // The planner only needs Costmap2DROS for the frame name; tests plan on
    // their own Costmap2D, injected with setTestCostmap().
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
  }

  static void TearDownTestSuite()
  {
    costmap_ros_.reset();
    node_.reset();
  }

  void SetUp() override
  {
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(kCells, kCells, kRes, 0.0, 0.0, 0);
  }

  /// Planner with the given parameter overrides, planning on costmap_.
  std::shared_ptr<LazyCoulombPlanner> makePlanner(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    const std::string name = "LCP" + std::to_string(counter_++);
    for (const auto & p : overrides) {
      node_->declare_parameter(name + "." + p.get_name(), p.get_parameter_value());
    }
    auto planner = std::make_shared<LazyCoulombPlanner>();
    planner->configure(node_, name, nullptr, costmap_ros_);
    planner->setTestCostmap(costmap_.get());
    return planner;
  }

  /// Fill the world-frame rectangle [x0, x1] x [y0, y1] with lethal cells.
  void addBox(double x0, double y0, double x1, double y1)
  {
    for (unsigned int my = 0; my < kCells; ++my) {
      for (unsigned int mx = 0; mx < kCells; ++mx) {
        const double wx = (mx + 0.5) * kRes, wy = (my + 0.5) * kRes;
        if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) {
          costmap_->setCost(mx, my, LETHAL_OBSTACLE);
        }
      }
    }
  }

  bool lethal(double wx, double wy) const
  {
    unsigned int mx, my;
    return !costmap_->worldToMap(wx, wy, mx, my) || costmap_->getCost(mx, my) >= 253;
  }

  /// Independent collision audit: sample every segment at a tenth of a cell.
  bool collides(const nav_msgs::msg::Path & path) const
  {
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      const int n = std::max(1, static_cast<int>(std::hypot(b.x - a.x, b.y - a.y) / (kRes / 10)));
      for (int k = 0; k <= n; ++k) {
        const double t = static_cast<double>(k) / n;
        if (lethal(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) {
          return true;
        }
      }
    }
    return false;
  }

  static double length(const nav_msgs::msg::Path & path)
  {
    double sum = 0.0;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      sum += std::hypot(b.x - a.x, b.y - a.y);
    }
    return sum;
  }

  /// Largest heading change between consecutive segments, in radians.
  static double sharpestCorner(const nav_msgs::msg::Path & path)
  {
    double worst = 0.0, prev = 0.0;
    bool have_prev = false;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) < 1e-9) {
        continue;
      }
      const double h = std::atan2(b.y - a.y, b.x - a.x);
      if (have_prev) {
        worst = std::max(worst, std::abs(std::atan2(std::sin(h - prev), std::cos(h - prev))));
      }
      prev = h;
      have_prev = true;
    }
    return worst;
  }

  static void expectEndpoints(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    ASSERT_GE(path.poses.size(), 2u);
    EXPECT_NEAR(path.poses.front().pose.position.x, start.pose.position.x, 1e-9);
    EXPECT_NEAR(path.poses.front().pose.position.y, start.pose.position.y, 1e-9);
    EXPECT_NEAR(path.poses.back().pose.position.x, goal.pose.position.x, 1e-9);
    EXPECT_NEAR(path.poses.back().pose.position.y, goal.pose.position.y, 1e-9);
  }

  static rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  static int counter_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
};

rclcpp_lifecycle::LifecycleNode::SharedPtr LcpTest::node_;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> LcpTest::costmap_ros_;
int LcpTest::counter_ = 0;

TEST_F(LcpTest, EmptyMapGivesStraightLine)
{
  auto planner = makePlanner();
  const auto start = pose(1.0, 1.0), goal = pose(9.0, 7.0, 1.0);
  const auto path = planner->createPlan(start, goal, never);

  expectEndpoints(path, start, goal);
  EXPECT_NEAR(length(path), std::hypot(8.0, 6.0), 1e-6);
  EXPECT_LT(sharpestCorner(path), 1e-6);
  EXPECT_EQ(path.header.frame_id, costmap_ros_->getGlobalFrameID());
  // Output is resampled at output_spacing (0.1 m by default).
  EXPECT_GE(path.poses.size(), 100u);
  // The last pose keeps the goal's orientation.
  EXPECT_NEAR(path.poses.back().pose.orientation.z, goal.pose.orientation.z, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.orientation.w, goal.pose.orientation.w, 1e-9);
}

TEST_F(LcpTest, GoesAroundSingleObstacle)
{
  addBox(4.0, 3.0, 6.0, 7.0);
  auto planner = makePlanner();
  const auto start = pose(1.0, 5.0), goal = pose(9.0, 5.0);
  const auto path = planner->createPlan(start, goal, never);

  expectEndpoints(path, start, goal);
  EXPECT_FALSE(collides(path));
  EXPECT_GT(length(path), 8.0);
  EXPECT_LT(length(path), 1.5 * 8.0);
}

TEST_F(LcpTest, PicksTheShorterSideOfAnOffCentreObstacle)
{
  // The obstacle sticks out much further below the straight line than above it.
  addBox(4.5, 1.0, 5.5, 5.5);
  auto planner = makePlanner();
  const auto path = planner->createPlan(pose(1.0, 5.0), pose(9.0, 5.0), never);

  EXPECT_FALSE(collides(path));
  double max_y = 0.0, min_y = 10.0;
  for (const auto & p : path.poses) {
    max_y = std::max(max_y, p.pose.position.y);
    min_y = std::min(min_y, p.pose.position.y);
  }
  EXPECT_GT(max_y, 5.5);  // went over the top
  EXPECT_GT(min_y, 4.0);  // and not the long way round
}

TEST_F(LcpTest, BothForceLawsAndProbeAvoidObstacles)
{
  addBox(3.0, 2.0, 4.0, 6.5);
  addBox(6.0, 4.0, 7.0, 9.0);
  const auto start = pose(1.0, 5.0), goal = pose(9.0, 5.0);
  const std::vector<std::vector<rclcpp::Parameter>> variants = {
    {rclcpp::Parameter("force_exponent", 1.0)},
    {rclcpp::Parameter("force_exponent", 2.0)},
    {rclcpp::Parameter("force_model", "probe")},
    {rclcpp::Parameter("commit_direction", false)},
  };
  for (const auto & overrides : variants) {
    auto planner = makePlanner(overrides);
    const auto path = planner->createPlan(start, goal, never);
    expectEndpoints(path, start, goal);
    EXPECT_FALSE(collides(path));
  }
}

TEST_F(LcpTest, RejectsBadStartAndGoal)
{
  addBox(4.0, 4.0, 6.0, 6.0);
  auto planner = makePlanner();
  EXPECT_THROW(
    planner->createPlan(pose(5.0, 5.0), pose(9.0, 9.0), never), nav2_core::StartOccupied);
  EXPECT_THROW(
    planner->createPlan(pose(1.0, 1.0), pose(5.0, 5.0), never), nav2_core::GoalOccupied);
  EXPECT_THROW(
    planner->createPlan(pose(-1.0, 1.0), pose(9.0, 9.0), never),
    nav2_core::StartOutsideMapBounds);
  EXPECT_THROW(
    planner->createPlan(pose(1.0, 1.0), pose(9.0, 11.0), never),
    nav2_core::GoalOutsideMapBounds);
}

TEST_F(LcpTest, StartJustInsideAnObstacleStepsOutFirst)
{
  // A robot that stopped slightly inside the inflated zone must still get a plan.
  addBox(4.0, 4.0, 6.0, 6.0);
  auto planner = makePlanner();
  const auto start = pose(4.2, 5.0), goal = pose(9.0, 9.0);
  auto path = planner->createPlan(start, goal, never);

  expectEndpoints(path, start, goal);
  // Only the first step, from the robot to free space, may touch the obstacle.
  path.poses.erase(path.poses.begin());
  EXPECT_FALSE(collides(path));

  auto strict = makePlanner({rclcpp::Parameter("start_escape_radius", 0.0)});
  EXPECT_THROW(strict->createPlan(start, goal, never), nav2_core::StartOccupied);
}

TEST_F(LcpTest, EnclosedGoalFailsInsteadOfReturningAPath)
{
  // A closed ring of walls around the goal: no route exists.
  addBox(6.0, 4.0, 8.0, 4.2);
  addBox(6.0, 5.8, 8.0, 6.0);
  addBox(6.0, 4.0, 6.2, 6.0);
  addBox(7.8, 4.0, 8.0, 6.0);
  auto planner = makePlanner();
  EXPECT_THROW(
    planner->createPlan(pose(1.0, 5.0), pose(7.0, 5.0), never),
    nav2_core::NoValidPathCouldBeFound);
}

TEST_F(LcpTest, RetriesTheOtherSideWhenTheFirstIsBlocked)
{
  // A wall from the bottom edge of the map up to y = 6, wider than the charge
  // radius, so the forces on a waypoint inside it cancel and the first push is
  // the default one: to the left of travel. Travelling right-to-left that means
  // downwards, which is sealed by the map edge. Only going over the top works.
  addBox(4.8, 0.0, 5.2, 6.0);
  const auto start = pose(9.0, 1.5), goal = pose(1.0, 1.5);

  auto with_retry = makePlanner();
  const auto path = with_retry->createPlan(start, goal, never);
  expectEndpoints(path, start, goal);
  EXPECT_FALSE(collides(path));

  auto without_retry = makePlanner({rclcpp::Parameter("retry_other_side", false)});
  EXPECT_THROW(
    without_retry->createPlan(start, goal, never), nav2_core::NoValidPathCouldBeFound);
}

TEST_F(LcpTest, CancelIsHonoured)
{
  addBox(4.0, 3.0, 6.0, 7.0);
  auto planner = makePlanner();
  EXPECT_THROW(
    planner->createPlan(pose(1.0, 5.0), pose(9.0, 5.0), []() {return true;}),
    nav2_core::PlannerCancelled);
}

TEST_F(LcpTest, SmoothingRoundsCornersWithoutColliding)
{
  addBox(4.0, 3.0, 6.0, 7.0);
  const auto start = pose(1.0, 5.0), goal = pose(9.0, 5.0);

  auto sharp = makePlanner({rclcpp::Parameter("enable_smoothing", false)});
  auto smooth = makePlanner();
  const auto sharp_path = sharp->createPlan(start, goal, never);
  const auto smooth_path = smooth->createPlan(start, goal, never);

  EXPECT_FALSE(collides(sharp_path));
  EXPECT_FALSE(collides(smooth_path));
  EXPECT_GT(sharpestCorner(sharp_path), 0.5);
  EXPECT_LT(sharpestCorner(smooth_path), 0.5 * sharpestCorner(sharp_path));
  EXPECT_LE(length(smooth_path), length(sharp_path) + 1e-9);
}

TEST_F(LcpTest, NeverReturnsACollidingPathInRandomClutter)
{
  // Property test: whatever the planner returns must be collision free, for
  // every force law, with and without a waypoint margin.
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> pos(0.5, 9.5), size(0.1, 0.6);
  for (int i = 0; i < 40; ++i) {
    const double x = pos(rng), y = pos(rng), w = size(rng), h = size(rng);
    addBox(x, y, x + w, y + h);
  }

  const std::vector<std::vector<rclcpp::Parameter>> variants = {
    {},
    {rclcpp::Parameter("force_exponent", 1.0)},
    {rclcpp::Parameter("force_model", "probe")},
    {rclcpp::Parameter("waypoint_margin", 0.0)},
    {rclcpp::Parameter("retry_order", "recent")},
  };
  for (const auto & overrides : variants) {
    auto planner = makePlanner(overrides);
    std::mt19937 pair_rng(11);
    int returned = 0, attempted = 0;
    while (attempted < 60) {
      const double sx = pos(pair_rng), sy = pos(pair_rng), gx = pos(pair_rng), gy = pos(pair_rng);
      if (lethal(sx, sy) || lethal(gx, gy)) {
        continue;
      }
      ++attempted;
      try {
        const auto path = planner->createPlan(pose(sx, sy), pose(gx, gy), never);
        ++returned;
        EXPECT_FALSE(collides(path)) << "from (" << sx << ", " << sy << ") to (" << gx << ", " <<
          gy << ")";
      } catch (const nav2_core::NoValidPathCouldBeFound &) {
        // Reporting failure is acceptable; a colliding path is not.
      }
    }
    EXPECT_GT(returned, 45);  // sparse clutter: most pairs must be solved
  }
}

TEST_F(LcpTest, RejectsUnknownOptions)
{
  EXPECT_THROW(makePlanner({rclcpp::Parameter("force_model", "magnetic")}), std::runtime_error);
  EXPECT_THROW(makePlanner({rclcpp::Parameter("retry_order", "random")}), std::runtime_error);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
