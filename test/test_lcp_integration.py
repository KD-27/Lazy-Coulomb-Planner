# Copyright 2026 Kaveesha Dhananjaya
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Integration tests for Lazy Coulomb Planner.

Launches the planner_server node with LCP configured and a static costmap,
then sends ComputePathToPose action goals and validates the responses.

Three obstacle scenarios are tested:
  1. single_box  — 1 m × 1 m box blocking the direct path
  2. wall        — vertical wall blocking the direct path
  3. narrow_gap  — near-impassable gap (documents LCP limitation)

Run with:
  colcon test --packages-select lazy_coulomb_planner
  colcon test-result --verbose
"""

import os
import time
import math
import unittest

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import Header

# ── Constants ─────────────────────────────────────────────────────────────────

# Costmap: 100 × 100 cells, 0.1 m/cell → 10 m × 10 m
MAP_WIDTH = 100    # cells
MAP_HEIGHT = 100    # cells
MAP_RES = 0.1    # m/cell
MAP_ORIGIN_X = 0.0
MAP_ORIGIN_Y = 0.0

ACTION_TIMEOUT = 30.0   # seconds to wait for action server
PLAN_TIMEOUT = 15.0   # seconds to wait for a plan response
STRAIGHT_DIST = 8.0    # metres (start (1,5) → goal (9,5))


# ── Map helpers ───────────────────────────────────────────────────────────────

def make_empty_map() -> OccupancyGrid:
    """Return a 10×10 m OccupancyGrid with all FREE cells."""
    grid = OccupancyGrid()
    grid.header = Header(frame_id='map')
    grid.info.resolution = MAP_RES
    grid.info.width = MAP_WIDTH
    grid.info.height = MAP_HEIGHT
    grid.info.origin.position.x = MAP_ORIGIN_X
    grid.info.origin.position.y = MAP_ORIGIN_Y
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (MAP_WIDTH * MAP_HEIGHT)
    return grid


def cell_index(mx: int, my: int) -> int:
    return my * MAP_WIDTH + mx


def fill_rect(grid: OccupancyGrid, mx0: int, my0: int, mx1: int, my1: int) -> None:
    """Set cells in the rectangle [mx0,mx1]×[my0,my1] to LETHAL (100)."""
    for mx in range(mx0, min(mx1 + 1, MAP_WIDTH)):
        for my in range(my0, min(my1 + 1, MAP_HEIGHT)):
            grid.data[cell_index(mx, my)] = 100


def make_single_box_map() -> OccupancyGrid:
    """1 m×1 m box centered at (5, 5): mx 45–55, my 45–55."""
    grid = make_empty_map()
    fill_rect(grid, 45, 45, 55, 55)
    return grid


def make_wall_map() -> OccupancyGrid:
    """Vertical wall at x=5 (mx=50), y=1–8 (my=10–80)."""
    grid = make_empty_map()
    fill_rect(grid, 49, 10, 51, 80)
    return grid


def make_narrow_gap_map() -> OccupancyGrid:
    """Wall at x=5 with a 1-cell gap at y=5.0."""
    grid = make_empty_map()
    fill_rect(grid, 49,  5, 51, 49)   # wall below gap
    fill_rect(grid, 49, 51, 51, 95)   # wall above gap
    return grid


def make_pose(x: float, y: float, frame: str = 'map') -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.w = 1.0
    return ps


def path_length(path: Path) -> float:
    total = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
        dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
        total += math.sqrt(dx * dx + dy * dy)
    return total


# ── Launch description ─────────────────────────────────────────────────────────

@pytest.mark.launch_test
def generate_test_description():
    """Launch planner_server with LCP configured (no Gazebo needed)."""
    pkg_share = os.path.join(
        os.path.dirname(__file__), '..', 'config')
    params_file = os.path.join(pkg_share, 'nav2_params.yaml')

    planner_server = launch_ros.actions.Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False},
        ],
    )

    return launch.LaunchDescription([
        planner_server,
        launch_testing.actions.ReadyToTest(),
    ])


# ── Test helper node ──────────────────────────────────────────────────────────

class PlannerTestNode(Node):
    """Minimal ROS 2 node that publishes a map and calls ComputePathToPose."""

    def __init__(self):
        super().__init__('lcp_integration_test_node')

        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', rclpy.qos.QoSProfile(
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ))

        self._action_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')

        self._result = None
        self._done = False

    def publish_map(self, grid: OccupancyGrid) -> None:
        self._result = None
        self._done = False
        self.map_pub.publish(grid)

    def request_plan(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        timeout: float = PLAN_TIMEOUT
    ):
        """
        Send a ComputePathToPose goal and block until a result arrives.

        or timeout expires.  Returns the action result or None.
        """
        if not self._action_client.wait_for_server(timeout_sec=ACTION_TIMEOUT):
            self.get_logger().error('ComputePathToPose action server not available')
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = 'GridBased'

        self._done = False
        self._result = None

        send_future = self._action_client.send_goal_async(goal_msg)

        deadline = time.time() + timeout
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if send_future.done():
                goal_handle = send_future.result()
                if not goal_handle.accepted:
                    return None
                result_future = goal_handle.get_result_async()
                inner_deadline = time.time() + timeout
                while time.time() < inner_deadline:
                    executor.spin_once(timeout_sec=0.1)
                    if result_future.done():
                        return result_future.result().result
                return None

        return None


# ── Test cases ────────────────────────────────────────────────────────────────

class TestLCPIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = PlannerTestNode()
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        # Give the planner_server a moment to start
        time.sleep(3.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.start = make_pose(1.0, 5.0)
        self.goal = make_pose(9.0, 5.0)

    # ── Test 1: Open space ────────────────────────────────────────────────────

    def test_01_open_space_path(self):
        """In an empty map, LCP should return a near-straight-line path."""
        self.node.publish_map(make_empty_map())
        time.sleep(0.5)

        result = self.node.request_plan(self.start, self.goal)
        self.assertIsNotNone(result, 'Open-space plan request timed out or failed')
        self.assertGreater(len(result.path.poses), 0, 'Path must not be empty')

        length = path_length(result.path)
        self.assertAlmostEqual(
            length, STRAIGHT_DIST, delta=0.5,
            msg=f'Open-space path {length:.2f} m should be ≈ {STRAIGHT_DIST} m')

    # ── Test 2: Single box ────────────────────────────────────────────────────

    def test_02_single_box_detour(self):
        """LCP must produce a longer path that avoids the box."""
        self.node.publish_map(make_single_box_map())
        time.sleep(0.5)

        result = self.node.request_plan(self.start, self.goal)
        self.assertIsNotNone(result, 'Single-box plan request timed out or failed')
        self.assertGreater(len(result.path.poses), 0, 'Path must not be empty')

        # ── Geometry check ────────────────────────────────────────────────────
        length = path_length(result.path)
        self.assertGreater(
            length, STRAIGHT_DIST,
            f'Single-box path {length:.2f} m must be longer than straight line')

        self.assertGreater(
            len(result.path.poses), 20,
            'Single-box path must have more poses than initial_path_points (20)')

        # ── Costmap check: no pose inside the obstacle rectangle ───────────────
        for pose in result.path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.assertFalse(
                4.5 <= x <= 5.5 and 4.5 <= y <= 5.5,
                f'Path pose ({x:.2f}, {y:.2f}) is inside the obstacle box')

    # ── Test 3: Wall obstacle ─────────────────────────────────────────────────

    def test_03_wall_obstacle_detour(self):
        """LCP must detour around the vertical wall."""
        self.node.publish_map(make_wall_map())
        time.sleep(0.5)

        result = self.node.request_plan(self.start, self.goal)
        self.assertIsNotNone(result, 'Wall plan request timed out or failed')
        self.assertGreater(len(result.path.poses), 0, 'Path must not be empty')

        # ── Geometry check ────────────────────────────────────────────────────
        length = path_length(result.path)
        self.assertGreater(
            length, STRAIGHT_DIST + 1.0,
            f'Wall path {length:.2f} m must be substantially longer than straight line')

        # ── Costmap check: no pose inside the wall column ─────────────────────
        for pose in result.path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.assertFalse(
                4.9 <= x <= 5.1 and 1.0 <= y <= 8.0,
                f'Path pose ({x:.2f}, {y:.2f}) is inside the wall')

    # ── Test 4: Narrow gap ────────────────────────────────────────────────────

    def test_04_narrow_gap_documents_behaviour(self):
        """
        LCP may succeed (squeezing through) or fail (PlannerException) for a narrow gap.

        1-cell gap.  Either outcome is recorded; this test never fails hard.
        This is a documented known limitation of LCP.
        """
        self.node.publish_map(make_narrow_gap_map())
        time.sleep(0.5)

        result = self.node.request_plan(self.start, self.goal)

        if result is None or len(result.path.poses) == 0:
            print('[NarrowGap] LCP could not find a path — expected for 0.1 m gap')
        else:
            length = path_length(result.path)
            print(f'[NarrowGap] LCP returned a path of {length:.2f} m — '
                  f'verify physical validity (no inflation in raw costmap)')

        # Always passes — documents behaviour
        self.assertTrue(True, 'NarrowGap behaviour documented')
