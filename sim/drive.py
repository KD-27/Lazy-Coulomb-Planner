#!/usr/bin/env python3
"""Drive the simulated robot through a fixed sequence of goals and record how it went.

Goals are sampled with a fixed seed from the static map (cells at least
CLEARANCE metres from any obstacle), so every planner gets the same sequence.
Usage:  drive.py <map.yaml> <out.csv> <label> <goals> <start_x> <start_y>
"""
import csv
import math
import os
import random
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import numpy as np
from PIL import Image
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan
import yaml

map_yaml, out_csv, label, n_goals = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
start = (float(sys.argv[5]), float(sys.argv[6]))
CLEARANCE, MIN_LEG, LEG_TIMEOUT = 0.3, 1.5, 120.0

meta = yaml.safe_load(open(map_yaml))
img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])))
res, (ox, oy, _) = meta['resolution'], meta['origin']
darkness = (255 - img) / 255.0
free = darkness < meta['free_thresh']
dist = ndimage.distance_transform_edt(free) * res   # metres to the nearest non-free cell
# Only cells connected to the start, so goals are not placed outside the arena.
labels, _ = ndimage.label(dist >= CLEARANCE)
start_rc = (img.shape[0] - 1 - int((start[1] - oy) / res), int((start[0] - ox) / res))
rows, cols = np.nonzero(labels == labels[start_rc])
assert labels[start_rc] != 0, 'start pose is not in free space on the map'

random.seed(5)
goals, prev = [], start
while len(goals) < n_goals:
    k = random.randrange(len(rows))
    g = (ox + (cols[k] + 0.5) * res, oy + (img.shape[0] - 1 - rows[k] + 0.5) * res)
    if math.hypot(g[0] - prev[0], g[1] - prev[1]) >= MIN_LEG:
        goals.append(g)
        prev = g


def pose(nav, x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
    return p


rclpy.init()
nav = BasicNavigator()
state = {'odom': None, 'travelled': 0.0, 'min_range': float('inf')}


def on_odom(msg):
    p = msg.pose.pose.position
    if state['odom'] is not None:
        state['travelled'] += math.hypot(p.x - state['odom'][0], p.y - state['odom'][1])
    state['odom'] = (p.x, p.y)


def on_scan(msg):
    valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
    if valid:
        state['min_range'] = min(state['min_range'], min(valid))


nav.create_subscription(Odometry, 'odom', on_odom, 10)
nav.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)

nav.setInitialPose(pose(nav, *start))
nav.waitUntilNav2Active()
time.sleep(3.0)

rows_out, here = [], start
for i, g in enumerate(goals):
    path = nav.getPath(pose(nav, *here), pose(nav, *g), use_start=False)
    planned = float('nan')
    if path is not None and len(path.poses) >= 2:
        planned = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                                 b.pose.position.y - a.pose.position.y)
                      for a, b in zip(path.poses, path.poses[1:]))
    state['travelled'], state['min_range'] = 0.0, float('inf')
    t0 = nav.get_clock().now()
    nav.goToPose(pose(nav, *g))
    recoveries = 0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None:
            recoveries = fb.number_of_recoveries
            if Duration.from_msg(fb.navigation_time) > Duration(seconds=LEG_TIMEOUT):
                nav.cancelTask()
    result = nav.getResult()
    sim_s = (nav.get_clock().now() - t0).nanoseconds / 1e9
    reached = result == TaskResult.SUCCEEDED
    rows_out.append(dict(
        planner=label, leg=i, goal_x=round(g[0], 3), goal_y=round(g[1], 3),
        straight_m=round(math.hypot(g[0] - here[0], g[1] - here[1]), 3),
        planned_m=round(planned, 3), reached=int(reached), sim_time_s=round(sim_s, 2),
        travelled_m=round(state['travelled'], 3), min_scan_range_m=round(state['min_range'], 3),
        recoveries=recoveries))
    print(rows_out[-1], flush=True)
    here = g if reached else (state['odom'] or here)

with open(out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows_out[0]))
    w.writeheader()
    w.writerows(rows_out)

ok = [r for r in rows_out if r['reached']]
print(f"\n{label}: reached {len(ok)}/{len(rows_out)} goals, "
      f"sim time {sum(r['sim_time_s'] for r in rows_out):.1f} s, "
      f"travelled {sum(r['travelled_m'] for r in rows_out):.2f} m, "
      f"closest obstacle {min(r['min_scan_range_m'] for r in rows_out):.3f} m, "
      f"recoveries {sum(r['recoveries'] for r in rows_out)}")
rclpy.shutdown()
