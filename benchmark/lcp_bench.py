#!/usr/bin/env python3
"""
Planner benchmark derived from Nav2's tools/planner_benchmarking/metrics.py.

Differences from upstream: failures are recorded instead of discarded, every
returned path is audited against the costmap at a tenth-of-a-cell spacing, and the map
origin is respected. Usage:  lcp_bench.py <out.csv> <pairs> <planner> [<planner> ...]
Clearance is measured in metres from the obstacle cells of the costmap (cost 254
and above, i.e. the map's own obstacles, not the inflation), by a Euclidean
distance transform sampled along the path at the same spacing: the distance from
the path to the nearest edge of an obstacle cell, for the robot's centre line.
The first planner is the reference for "a route exists" and for path length.
Set LCP_BENCH_REPEATS to plan every pair that many times; the planning time
reported is then the median of the repeats (the fastest and slowest are kept
too), and the path is the one from the first call.
"""
import csv
import math
import os
import random
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from rclpy.action import ActionClient
from scipy.ndimage import distance_transform_edt

out_csv, n_pairs, planners = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
MAX_START_COST, MIN_DIST, OBSTACLE_COST = 210, 3.0, 253
NEAR_M = 0.5  # a path sample closer than this to an obstacle counts as "near"
REPEATS = max(1, int(os.environ.get('LCP_BENCH_REPEATS', '1')))

rclpy.init()
nav = BasicNavigator()
time.sleep(2.0)
cm = nav.getGlobalCostmap()
res, ox, oy = cm.metadata.resolution, cm.metadata.origin.position.x, cm.metadata.origin.position.y
grid = np.asarray(cm.data, dtype=np.uint8).reshape(cm.metadata.size_y, cm.metadata.size_x)
assert (grid >= 254).any(), 'costmap has no obstacles - map not loaded?'
free_rows, free_cols = np.nonzero(grid < MAX_START_COST)
# Distance from each cell centre to the nearest obstacle cell centre, less half a
# cell: roughly the distance to that obstacle cell's edge.
clear = np.maximum(distance_transform_edt(grid < 254) * res - 0.5 * res, 0.0)

ac = ActionClient(nav, ComputePathToPose, 'compute_path_to_pose')
assert ac.wait_for_server(timeout_sec=30.0)


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p


def random_free():
    k = random.randrange(len(free_rows))
    return ox + (free_cols[k] + 0.5) * res, oy + (free_rows[k] + 0.5) * res


def plan(planner, s, g):
    goal = ComputePathToPose.Goal()
    goal.start, goal.goal, goal.planner_id, goal.use_start = pose(*s), pose(*g), planner, True
    f = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(nav, f)
    rf = f.result().get_result_async()
    rclpy.spin_until_future_complete(nav, rf)
    return rf.result().result


def timed_plan(planner, s, g):
    t0 = time.perf_counter()
    r = plan(planner, s, g)
    return r, (time.perf_counter() - t0) * 1e3


def audit(pts):
    """Path length, costs, heading change (rad) and clearance (m) along a path.

    Returns length, highest cost, total turn, sharpest turn, mean cost, smallest
    clearance, mean clearance and the fraction of samples nearer than NEAR_M.
    """
    length, worst, turn, sharpest, prev = 0.0, 0, 0.0, 0.0, None
    cost_sum, cost_n = 0.0, 0
    clear_min, clear_sum, near_n = float('inf'), 0.0, 0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-9:
            continue
        length += d
        h = math.atan2(y1 - y0, x1 - x0)
        if prev is not None:
            dh = abs(math.atan2(math.sin(h - prev), math.cos(h - prev)))
            turn += dh
            sharpest = max(sharpest, dh)
        prev = h
        t = np.linspace(0.0, 1.0, max(2, int(d / (res / 10)) + 1))
        c = np.floor((x0 + (x1 - x0) * t - ox) / res).astype(int)
        r = np.floor((y0 + (y1 - y0) * t - oy) / res).astype(int)
        inside = (r >= 0) & (r < grid.shape[0]) & (c >= 0) & (c < grid.shape[1])
        if not inside.all():
            worst = 255
        costs = grid[r[inside], c[inside]]
        worst = max(worst, int(costs.max()))
        cost_sum += float(costs.sum())
        cost_n += len(costs)
        if len(costs):
            d_clear = clear[r[inside], c[inside]]
            clear_min = min(clear_min, float(d_clear.min()))
            clear_sum += float(d_clear.sum())
            near_n += int((d_clear < NEAR_M).sum())
    n = max(cost_n, 1)
    return (length, worst, turn, sharpest, cost_sum / n,
            clear_min if cost_n else float('nan'), clear_sum / n, near_n / n)


random.seed(33)
rows = []
for pair in range(n_pairs):
    s = random_free()
    while True:
        g = random_free()
        if math.hypot(g[0] - s[0], g[1] - s[1]) > MIN_DIST:
            break
    for planner in planners:
        r, wall_ms = timed_plan(planner, s, g)
        times = [r.planning_time.sec * 1e3 + r.planning_time.nanosec / 1e6]
        for _ in range(REPEATS - 1):
            again, _ = timed_plan(planner, s, g)
            times.append(again.planning_time.sec * 1e3 + again.planning_time.nanosec / 1e6)
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
        ok = r.error_code == 0 and len(pts) >= 2
        nan = float('nan')
        (length, worst, turn, sharpest, mean_cost, clear_min, clear_mean,
         near_frac) = audit(pts) if ok else (nan, -1, nan, nan, nan, nan, nan, nan)
        rows.append({
            'pair': pair, 'planner': planner, 'sx': s[0], 'sy': s[1], 'gx': g[0], 'gy': g[1],
            'straight': math.hypot(g[0] - s[0], g[1] - s[1]), 'error_code': r.error_code,
            'returned': int(ok), 'valid': int(ok and worst < OBSTACLE_COST),
            'time_ms': float(np.median(times)), 'time_min_ms': min(times),
            'time_max_ms': max(times), 'repeats': REPEATS,
            'wall_ms': wall_ms, 'length': length, 'max_cost': worst, 'mean_cost': mean_cost,
            'turn_rad': turn,
            'sharpest_deg': math.degrees(sharpest), 'poses': len(pts),
            'clear_min_m': clear_min, 'clear_mean_m': clear_mean, 'near_frac': near_frac})

with open(out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

ref = {r['pair']: r for r in rows if r['planner'] == planners[0]}
solvable = {p for p, r in ref.items() if r['valid']}
print(f'\n{n_pairs} pairs, {len(solvable)} with a route according to {planners[0]}')
header = 'planner'
columns = ['valid', 'collide', 'failed', 'false+', 't_med', 't_p95', 't_max', 'failwall',
           'len/ref', 'turn', 'corner', 'maxcost', 'meancost', 'clr_min', 'clr_mean']
print(f'{header:>12} ' + ' '.join(f'{c:>8}' for c in columns))


def median(values):
    return float(np.median(values)) if values else float('nan')


for planner in planners:
    mine = [r for r in rows if r['planner'] == planner]
    on = [r for r in mine if r['pair'] in solvable]
    valid = [r for r in on if r['valid']]
    collide = sum(r['returned'] and not r['valid'] for r in on)
    failed = sum(not r['returned'] for r in on)
    false_pos = sum(r['returned'] for r in mine if r['pair'] not in solvable)
    # planner_server reports no planning time for failed requests, so the time
    # columns cover returned paths only.
    t = np.array([r['time_ms'] for r in mine if r['returned']] or [float('nan')])
    ratio = median([r['length'] / ref[r['pair']]['length'] for r in valid])
    turn = median([r['turn_rad'] for r in valid])
    corner = median([r['sharpest_deg'] for r in valid])
    # Slowest failed request, client round trip (includes ~1 ms of ROS overhead).
    fail_wall = max([r['wall_ms'] for r in mine if not r['returned']] or [float('nan')])
    cells = [f'{len(valid)}/{len(on)}', collide, failed, false_pos,
             f'{np.median(t):.2f}', f'{np.percentile(t, 95):.2f}', f'{t.max():.2f}',
             f'{fail_wall:.1f}', f'{ratio:.3f}', f'{turn:.2f}', f'{corner:.1f}',
             f'{median([r["max_cost"] for r in valid]):.0f}',
             f'{median([r["mean_cost"] for r in valid]):.1f}',
             f'{median([r["clear_min_m"] for r in valid]):.2f}',
             f'{median([r["clear_mean_m"] for r in valid]):.2f}']
    print(f'{planner:>12} ' + ' '.join(f'{c:>8}' for c in cells))
rclpy.shutdown()
