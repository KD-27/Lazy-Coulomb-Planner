#!/usr/bin/env python3
"""
Drive the simulated robot through a fixed sequence of goals and record how it went.

Goals are sampled with a fixed seed from the static map (cells at least
CLEARANCE metres from any obstacle), so every planner gets the same sequence.
Besides the CSV it writes <out>.trace.jsonl: the robot's pose five times a
second, every path the planner publishes, the velocity commands, and the
warnings of the controller (such as "collision ahead"), all with wall-clock
times, so a stop can be examined afterwards.
Usage:  drive.py <map.yaml> <out.csv> <label> <goals> <start_x> <start_y>
"""
import csv
import json
import math
import os
import random
import sys
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
import numpy as np
from PIL import Image
from rcl_interfaces.msg import Log
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import yaml

map_yaml, out_csv, label, n_goals = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
start = (float(sys.argv[5]), float(sys.argv[6]))
CLEARANCE, MIN_LEG = 0.3, 1.5
LEG_TIMEOUT = float(os.environ.get('LCP_SIM_LEG_TIMEOUT', '120'))   # simulated seconds per leg

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

# ---- trace ------------------------------------------------------------------
trace = open(out_csv.rsplit('.', 1)[0] + '.trace.jsonl', 'w')
state['leg'] = -1
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, nav)


def record(kind, **fields):
    trace.write(json.dumps({'t': round(time.time(), 3), 'leg': state['leg'], 'kind': kind,
                            **fields}) + '\n')


def on_pose_timer():
    try:
        tf = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    except Exception:  # noqa: B902  the transform is simply not there yet
        return
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    record('pose', x=round(tf.transform.translation.x, 3),
           y=round(tf.transform.translation.y, 3), yaw=round(yaw, 3))


def on_plan(msg):
    pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses[::2]]
    record('plan', points=pts)


def on_cmd(msg):
    record('cmd', v=round(msg.linear.x, 3), w=round(msg.angular.z, 3))


def on_log(msg):
    if msg.level >= 30 and msg.name in ('controller_server', 'planner_server', 'bt_navigator',
                                        'behavior_server'):
        record('log', node=msg.name, text=msg.msg[:160])


nav.create_timer(0.2, on_pose_timer)
nav.create_subscription(Path, 'plan', on_plan, 10)
nav.create_subscription(Twist, 'cmd_vel', on_cmd, 10)
nav.create_subscription(Log, 'rosout', on_log, 100)
# -----------------------------------------------------------------------------

if os.environ.get('LCP_SIM_LOCALIZATION') == 'odometry':
    nav.waitUntilNav2Active(localizer='map_server')   # no AMCL, no initial pose needed
else:
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
    state['leg'] = i
    record('goal', x=round(g[0], 3), y=round(g[1], 3))
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
    rows_out.append({
        'planner': label, 'leg': i, 'goal_x': round(g[0], 3), 'goal_y': round(g[1], 3),
        'straight_m': round(math.hypot(g[0] - here[0], g[1] - here[1]), 3),
        'planned_m': round(planned, 3), 'reached': int(reached), 'sim_time_s': round(sim_s, 2),
        'travelled_m': round(state['travelled'], 3),
        'min_scan_range_m': round(state['min_range'], 3), 'recoveries': recoveries})
    print(rows_out[-1], flush=True)
    here = g if reached else (state['odom'] or here)

trace.close()
with open(out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows_out[0]))
    w.writeheader()
    w.writerows(rows_out)

reached_count = sum(r['reached'] for r in rows_out)
sim_time = sum(r['sim_time_s'] for r in rows_out)
travelled = sum(r['travelled_m'] for r in rows_out)
closest = min(r['min_scan_range_m'] for r in rows_out)
recovery_count = sum(r['recoveries'] for r in rows_out)
print(f'\n{label}: reached {reached_count}/{len(rows_out)} goals, sim time {sim_time:.1f} s, '
      f'travelled {travelled:.2f} m, closest obstacle {closest:.3f} m, '
      f'recoveries {recovery_count}')
rclpy.shutdown()
