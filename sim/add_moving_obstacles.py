#!/usr/bin/env python3
"""
Add obstacles that move back and forth on straight lanes to a Gazebo world.

Usage: add_moving_obstacles.py <world.sdf> <map.yaml> <out prefix>
           --count 6 --speed 0.3 --start X Y [--seed 1]
Each obstacle is an upright cylinder (radius 0.25 m, 0.5 m tall) driven by
Gazebo's velocity-control system. A lane is a straight segment, 3 to 6 m long,
whose cells all have at least LANE_CLEARANCE of room, and which keeps 1.5 m away
from the robot's start. Writes:
  <out>.sdf          the world with the moving obstacles
  <out>.lanes.yaml   the lanes, read by move_obstacles.py
  <out>.bridge.yaml  ros_gz_bridge topics for their velocity commands
The moving obstacles are not in the map: Nav2 sees them only through the lidar.
"""
import argparse
import math

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

RADIUS, HEIGHT, MASS = 0.25, 0.5, 200.0
LANE_CLEARANCE = RADIUS + 0.25

parser = argparse.ArgumentParser()
parser.add_argument('world')
parser.add_argument('map')
parser.add_argument('out')
parser.add_argument('--count', type=int, default=6)
parser.add_argument('--speed', type=float, default=0.3, help='m/s')
parser.add_argument('--start', type=float, nargs=2, required=True)
parser.add_argument('--seed', type=int, default=1)
args = parser.parse_args()

meta = yaml.safe_load(open(args.map))
res, (ox, oy, _) = meta['resolution'], meta['origin']
occupied = np.array(Image.open(args.map.rsplit('/', 1)[0] + '/' + meta['image']))[::-1] < 128
clear = ndimage.distance_transform_edt(~occupied) * res
rng = np.random.default_rng(args.seed)
free = np.argwhere(clear >= LANE_CLEARANCE)


def lane_ok(a, b):
    n = max(2, int(math.dist(a, b) / (res / 2)))
    for t in np.linspace(0.0, 1.0, n):
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        c, r = int((x - ox) / res), int((y - oy) / res)
        if (
            not (0 <= r < clear.shape[0] and 0 <= c < clear.shape[1])
            or clear[r, c] < LANE_CLEARANCE
        ):
            return False
        if math.dist((x, y), args.start) < 1.5:
            return False
    return True


lanes = []
for _ in range(20000):
    if len(lanes) == args.count:
        break
    r, c = free[rng.integers(len(free))]
    a = (ox + (c + 0.5) * res, oy + (r + 0.5) * res)
    angle, length = rng.uniform(0, math.pi), rng.uniform(3.0, 6.0)
    b = (a[0] + length * math.cos(angle), a[1] + length * math.sin(angle))
    if not lane_ok(a, b):
        continue
    # Keep lanes apart so obstacles do not collide with each other.
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    if any(
        math.dist(mid, ((lane['a'][0] + lane['b'][0]) / 2, (lane['a'][1] + lane['b'][1]) / 2))
        < 2.5
        for lane in lanes
    ):
        continue
    lanes.append(
        {
            'name': f'mover_{len(lanes)}',
            'a': [round(float(v), 3) for v in a],
            'b': [round(float(v), 3) for v in b],
            'speed': args.speed,
        }
    )
if len(lanes) < args.count:
    raise SystemExit(f'only found {len(lanes)} lanes, asked for {args.count}')

inertia_xx = MASS * (3 * RADIUS**2 + HEIGHT**2) / 12
inertia_zz = MASS * RADIUS**2 / 2
models = []
for lane in lanes:
    (ax, ay), (bx, by) = lane['a'], lane['b']
    yaw = math.atan2(by - ay, bx - ax)
    geometry = f'<cylinder><radius>{RADIUS}</radius><length>{HEIGHT}</length></cylinder>'
    models.append(
        f"""    <model name="{lane['name']}">
      <pose>{ax} {ay} {HEIGHT / 2 + 0.005} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <inertial><mass>{MASS}</mass><inertia><ixx>{inertia_xx:.3f}</ixx><iyy>{inertia_xx:.3f}</iyy>
          <izz>{inertia_zz:.3f}</izz></inertia></inertial>
        <collision name="collision"><geometry>{geometry}</geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry>{geometry}</geometry>
          <material><diffuse>0.8 0.2 0.2 1</diffuse></material></visual>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <topic>/model/{lane['name']}/cmd_vel</topic>
      </plugin>
    </model>"""
    )
world = open(args.world).read()
assert '</world>' in world
world = world.replace('</world>', '\n'.join(models) + '\n  </world>')
open(args.out + '.sdf', 'w').write(world)
yaml.safe_dump({'lanes': lanes}, open(args.out + '.lanes.yaml', 'w'), sort_keys=False)
bridge = [
    {
        'topic_name': f"/model/{lane['name']}/cmd_vel",
        'ros_type_name': 'geometry_msgs/msg/Twist',
        'gz_type_name': 'gz.msgs.Twist',
        'direction': 'ROS_TO_GZ',
    }
    for lane in lanes
]
yaml.safe_dump(bridge, open(args.out + '.bridge.yaml', 'w'), sort_keys=False)
print(f'{args.out}.sdf: {len(lanes)} moving obstacles at {args.speed} m/s')
