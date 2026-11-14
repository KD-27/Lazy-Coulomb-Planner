#!/usr/bin/env python3
"""
Measure how much a planner's path changes when it replans while driving.

Nav2 replans about once a second from the robot's current pose. For every leg
of the driving test this plans once from the start, then replans from points
0.3 m apart along that first path, each shifted 5 cm sideways (alternating
sides) as a stand-in for tracking error. A replan is compared with the part of
the first path still ahead: `deviation` is the largest distance from the new
path's first 1.5 m to the old path.

Needs a running planner_server (see leg_paths.py). Use a ROS domain of its own.
Usage: replan_stability.py <driving csv> <out.csv> <planner>...
"""
import csv
import sys

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

legs_csv, out_csv, planners = sys.argv[1], sys.argv[2], sys.argv[3:]
goals = [(float(r['goal_x']), float(r['goal_y'])) for r in csv.DictReader(open(legs_csv))]
starts = [(-2.0, -0.5)] + goals[:-1]
STEP, SHIFT, HORIZON, JUMP = 0.3, 0.05, 1.5, 0.3

rclpy.init()
node = Node('replan_stability')
ac = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
assert ac.wait_for_server(timeout_sec=30.0)


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p


def plan(planner, s, g):
    goal = ComputePathToPose.Goal()
    goal.start, goal.goal, goal.planner_id, goal.use_start = pose(*s), pose(*g), planner, True
    f = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f)
    rf = f.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    poses = rf.result().result.path.poses
    return np.array([(p.pose.position.x, p.pose.position.y) for p in poses])


def resample(pts, step=0.02):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.arange(0.0, s[-1], step)
    return np.column_stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])]), t


rows = []
for leg, (s, g) in enumerate(zip(starts, goals)):
    for planner in planners:
        first = plan(planner, s, g)
        if len(first) < 2:
            continue
        dense, arc = resample(first)
        k = 1
        while k * STEP < arc[-1] - 0.3:
            i = int(np.searchsorted(arc, k * STEP))
            heading = dense[min(i + 5, len(dense) - 1)] - dense[max(i - 5, 0)]
            heading /= np.linalg.norm(heading) + 1e-12
            side = 1.0 if k % 2 else -1.0
            start = dense[i] + side * SHIFT * np.array([-heading[1], heading[0]])
            again = plan(planner, tuple(start), g)
            if len(again) >= 2:
                new_dense, new_arc = resample(again)
                ahead = dense[i:]
                head = new_dense[new_arc <= HORIZON]
                d = np.min(np.linalg.norm(head[:, None, :] - ahead[None, :, :], axis=2), axis=1)
                rows.append({'leg': leg, 'planner': planner, 'along_m': round(k * STEP, 2),
                             'deviation_m': round(float(d.max()), 3), 'replanned': 1})
            else:
                rows.append({'leg': leg, 'planner': planner, 'along_m': round(k * STEP, 2),
                             'deviation_m': float('nan'), 'replanned': 0})
            k += 1

with open(out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

print(f"{'planner':>8} {'replans':>7} {'failed':>6} {'median dev':>10} {'p90 dev':>8} "
      f"{'jumps >' + str(JUMP) + ' m':>12} {'legs with a jump'}")
for planner in planners:
    mine = [r for r in rows if r['planner'] == planner]
    ok = [r['deviation_m'] for r in mine if r['replanned']]
    jumps = [r for r in mine if r['replanned'] and r['deviation_m'] > JUMP]
    legs = sorted({r['leg'] for r in jumps})
    print(f'{planner:>8} {len(mine):>7} {len(mine) - len(ok):>6} {np.median(ok):>10.3f} '
          f'{np.percentile(ok, 90):>8.3f} {len(jumps):>12} {legs}')
rclpy.shutdown()
