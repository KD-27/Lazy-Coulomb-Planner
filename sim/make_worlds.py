#!/usr/bin/env python3
"""
Build a set of Gazebo worlds from generated maps for the driving test.

Usage: make_worlds.py <set.yaml> <out_dir>
The set file lists worlds with the same keys as a benchmark suite's maps
(name, size, shape, density, seed, block, corridor). For each it writes, in
<out_dir>: <name>.pgm, <name>.yaml (the Nav2 map), <name>.sdf (the Gazebo world)
and adds a line to worlds.csv with the robot's start position, the free cell
with at least START_CLEARANCE of room that lies nearest the map centre.
"""
import csv
import os
import subprocess
import sys

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

START_CLEARANCE = 0.6  # m from the nearest obstacle

set_file, out_dir = sys.argv[1], sys.argv[2]
here = os.path.dirname(os.path.abspath(__file__))
generator = os.path.join(here, '..', 'benchmark', 'generate_map.py')
os.makedirs(out_dir, exist_ok=True)
rows = []
for spec in yaml.safe_load(open(set_file))['worlds']:
    prefix = os.path.join(out_dir, spec['name'])
    args = [
        '--size',
        str(spec['size']),
        '--shape',
        spec['shape'],
        '--seed',
        str(spec.get('seed', 1)),
        '--sdf',
    ]
    for key in ('density', 'block', 'corridor'):
        if key in spec:
            args += ['--' + key, str(spec[key])]
    made = (
        subprocess.run(
            [sys.executable, generator, prefix] + args, check=True, capture_output=True, text=True
        )
        .stdout.strip()
        .splitlines()
    )
    meta = yaml.safe_load(open(prefix + '.yaml'))
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    occupied = np.array(Image.open(prefix + '.pgm'))[::-1] < 128  # row 0 = lowest y
    clearance = ndimage.distance_transform_edt(~occupied) * res
    candidates = np.argwhere(clearance >= START_CLEARANCE)
    centre = np.array(occupied.shape) / 2.0
    r, c = candidates[np.argmin(((candidates - centre) ** 2).sum(axis=1))]
    start = (ox + (c + 0.5) * res, oy + (r + 0.5) * res)
    rows.append(
        {
            'world': spec['name'],
            'start_x': round(start[0], 3),
            'start_y': round(start[1], 3),
            'arguments': ' '.join(args),
            'obstacles': made[-1].split(': ', 1)[-1],
        }
    )
    print(f"{spec['name']}: start ({start[0]:.2f}, {start[1]:.2f}), {rows[-1]['obstacles']}")
with open(os.path.join(out_dir, 'worlds.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
