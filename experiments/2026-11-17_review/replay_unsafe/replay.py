#!/usr/bin/env python3
"""
Replay start/goal requests on a running planner server.

Saves each path with the cells (cost >= 253) it touches, sampled at a tenth of a cell
as lcp_bench does.

Usage: replay.py <out.json> <planner> sx,sy,gx,gy [...]
"""
import json
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from rclpy.action import ActionClient

out, planner = sys.argv[1], sys.argv[2]
reqs = [list(map(float, a.split(','))) for a in sys.argv[3:]]
rclpy.init()
nav = BasicNavigator()
time.sleep(2.0)
cm = nav.getGlobalCostmap()
res, ox, oy = cm.metadata.resolution, cm.metadata.origin.position.x, cm.metadata.origin.position.y
grid = np.asarray(cm.data, dtype=np.uint8).reshape(cm.metadata.size_y, cm.metadata.size_x)
ac = ActionClient(nav, ComputePathToPose, 'compute_path_to_pose')
assert ac.wait_for_server(timeout_sec=30.0)


def pose(x, y):
    """Return a map-frame pose."""
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p


results = []
for sx, sy, gx, gy in reqs:
    goal = ComputePathToPose.Goal(start=pose(sx, sy), goal=pose(gx, gy), planner_id=planner,
                                  use_start=True)
    f = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(nav, f)
    rf = f.result().get_result_async()
    rclpy.spin_until_future_complete(nav, rf)
    r = rf.result().result
    pts = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
    bad = []
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        for t in np.linspace(0.0, 1.0, max(2, int(d / (res / 10)) + 1)):
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            c, rr = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
            if grid[rr, c] >= 253:
                bad.append({'seg': i, 't': float(t), 'x': x, 'y': y, 'cell': [c, rr],
                            'cost': int(grid[rr, c]), 'a': pts[i], 'b': pts[i + 1]})
    # neighbourhood of the first bad cell
    hood = None
    if bad:
        c, rr = bad[0]['cell']
        hood = grid[rr - 3:rr + 4, c - 3:c + 4][::-1].tolist()  # rows top (high y) first
    results.append({'request': [sx, sy, gx, gy], 'error_code': r.error_code,
                    'poses': len(pts), 'bad_samples': len(bad), 'first_bad': bad[:3],
                    'bad_cells': sorted({tuple(b['cell']) for b in bad}),
                    'hood_7x7': hood, 'path': pts})
    print(sx, sy, gx, gy, 'error', r.error_code, 'poses', len(pts), 'bad samples', len(bad),
          'cells', sorted({tuple(b['cell']) for b in bad}), flush=True)
json.dump(results, open(out, 'w'))
rclpy.shutdown()
