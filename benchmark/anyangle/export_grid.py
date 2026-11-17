#!/usr/bin/env python3
"""
Export a benchmark map and its start/goal pairs for AnyAngleBench.java.

Usage: export_grid.py <map.yaml> <bench.csv> <out_prefix> [robot_radius_m]
Writes <out_prefix>.grid.txt and <out_prefix>.queries.csv.

The grid reproduces what LCP plans on: a tile is blocked when the Nav2 costmap
would give it cost 253 or more, i.e. it is an obstacle or unknown cell, or its
centre is within the robot radius (inscribed radius, 0.22 m in the benchmark)
of an obstacle cell centre. Start and goal are the pairs of the benchmark CSV,
rounded to the nearest grid vertex (the any-angle planners work on cell corners).
"""
import csv
import os
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

map_yaml, bench_csv, prefix = sys.argv[1:4]
radius = float(sys.argv[4]) if len(sys.argv) > 4 else 0.22

meta = yaml.safe_load(open(map_yaml))
res = float(meta['resolution'])
ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
img_path = os.path.join(os.path.dirname(map_yaml), meta['image'])


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # magic, width, height, maxval (comments skipped)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    pos += 1
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    assert tokens[0] == b'P5' and maxval < 256, 'only binary 8-bit PGM supported'
    return np.frombuffer(data, np.uint8, w * h, pos).reshape(h, w)


img = read_pgm(img_path).astype(float)
p = (255.0 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
occupied = p > float(meta['occupied_thresh'])
unknown = (p >= float(meta['free_thresh'])) & ~occupied
occupied, unknown = occupied[::-1], unknown[::-1]  # row 0 = y origin, as in the costmap
dist_cells = distance_transform_edt(~occupied)
blocked = occupied | unknown | (dist_cells * res <= radius)

h, w = blocked.shape
with open(prefix + '.grid.txt', 'w') as f:
    f.write(f'{w} {h}\n')
    for row in blocked:
        f.write(''.join('1' if b else '0' for b in row) + '\n')

seen = {}
for r in csv.DictReader(open(bench_csv)):
    seen.setdefault(int(r['pair']), r)
with open(prefix + '.queries.csv', 'w') as f:
    f.write('pair,sx,sy,gx,gy\n')
    for pair, r in sorted(seen.items()):
        v = [round((float(r[k]) - o) / res) for k, o in
             (('sx', ox), ('sy', oy), ('gx', ox), ('gy', oy))]
        f.write(f'{pair},{v[0]},{v[1]},{v[2]},{v[3]}\n')
print(f'{prefix}: {w}x{h} tiles, {blocked.mean() * 100:.1f} % blocked, {len(seen)} pairs')
