#!/usr/bin/env python3
"""
Plan every leg of the driving test with each planner and compare the path shapes.

Needs a running planner_server (benchmark/config/bringup_headless.launch.py with
benchmark/config/planners.yaml and sim/maps/tb3_sandbox.yaml). Writes a CSV with
per-leg clearance and curvature figures and a PNG of all paths on the map.
Usage: leg_paths.py <driving csv with the goal sequence> <map.yaml> <out prefix> <planner>...
"""
import csv
import math
import os
import sys

from geometry_msgs.msg import PoseStamped
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from scipy import ndimage  # noqa: E402
import yaml  # noqa: E402

legs_csv, map_yaml, out, planners = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
CLOSE = 0.33   # the robot stops for "collision ahead" when it is about this near an obstacle
meta = yaml.safe_load(open(map_yaml))
img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])))
res, (ox, oy, _) = meta['resolution'], meta['origin']
occupied = (255 - img) / 255.0 > meta['occupied_thresh']
dist = ndimage.distance_transform_edt(~occupied) * res


def clearance(x, y):
    r = img.shape[0] - 1 - int((y - oy) / res)
    c = int((x - ox) / res)
    return float(dist[r, c]) if 0 <= r < img.shape[0] and 0 <= c < img.shape[1] else 0.0


goals = [(float(r['goal_x']), float(r['goal_y'])) for r in csv.DictReader(open(legs_csv))]
starts = [(-2.0, -0.5)] + goals[:-1]

rclpy.init()
node = Node('leg_paths')
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
    return [(p.pose.position.x, p.pose.position.y) for p in rf.result().result.path.poses]


def resample(pts, step=0.05):
    out_pts, carry = [pts[0]], 0.0
    for a, b in zip(pts, pts[1:]):
        d = math.dist(a, b)
        t = step - carry
        while t <= d:
            out_pts.append((a[0] + (b[0] - a[0]) * t / d, a[1] + (b[1] - a[1]) * t / d))
            t += step
        carry = d - (t - step)
    return out_pts


def shape(pts):
    """Clearance figures, and the tightest turn radius measured over a 0.3 m window."""
    q = resample(pts)
    clear = [clearance(*p) for p in q]
    tight, tight_clear, w = float('inf'), float('nan'), 3
    for i in range(w, len(q) - w):
        h1 = math.atan2(q[i][1] - q[i - w][1], q[i][0] - q[i - w][0])
        h2 = math.atan2(q[i + w][1] - q[i][1], q[i + w][0] - q[i][0])
        dh = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        radius = (w * 0.05) / dh if dh > 1e-6 else float('inf')
        if radius < tight:
            tight, tight_clear = radius, clear[i]
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    close = 0.05 * sum(c < CLOSE for c in clear)   # samples are 0.05 m apart
    return length, min(clear), float(np.median(clear)), close, tight, tight_clear


rows = []
fig, axes = plt.subplots(3, 5, figsize=(20, 12))
extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
for leg, (s, g) in enumerate(zip(starts, goals)):
    ax = axes[leg // 5][leg % 5]
    ax.imshow(img, cmap='gray', extent=extent)
    for planner, colour in zip(planners, ['tab:red', 'tab:blue', 'tab:green']):
        pts = plan(planner, s, g)
        if len(pts) < 2:
            continue
        length, min_clear, median_clear, close, radius, radius_clear = shape(pts)
        rows.append({'leg': leg, 'planner': planner, 'length_m': round(length, 3),
                     'min_clearance_m': round(min_clear, 3),
                     'median_clearance_m': round(median_clear, 3),
                     'length_closer_than_%.2f_m' % CLOSE: round(close, 2),
                     'tightest_radius_m': round(radius, 3),
                     'clearance_at_tightest_m': round(radius_clear, 3)})
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colour, lw=1.5, label=planner)
    ax.plot(*s, 'ko', ms=5)
    ax.plot(*g, 'k*', ms=9)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_title(f'leg {leg}')
    ax.set_xticks([])
    ax.set_yticks([])
axes[0][0].legend(loc='lower left', fontsize=8)
fig.tight_layout()
fig.savefig(out + '.png', dpi=70)
with open(out + '.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
for r in rows:
    print(r)
key = 'length_closer_than_%.2f_m' % CLOSE
for planner in planners:
    mine = [r for r in rows if r['planner'] == planner]
    total = sum(r['length_m'] for r in mine)
    close = sum(r[key] for r in mine)
    typical = np.median([r['median_clearance_m'] for r in mine])
    radius = np.median([r['tightest_radius_m'] for r in mine])
    print(f'TOTAL {planner} length {total:.1f} m | closer than {CLOSE:.2f} m: {close:.1f} m '
          f'| median clearance {typical:.2f} m | tightest radius, median leg {radius:.2f} m')
rclpy.shutdown()
