#!/usr/bin/env python3
"""
Generate an occupancy map with controlled obstacle density, shape and size.

Random maps place obstacles of one shape until the requested fraction of the map
is occupied. `blocks` maps are a regular grid of square blocks separated by
corridors of a fixed width. The same arguments and seed always give the same map.

Usage:
  generate_map.py <out prefix> --size 50 --density 0.10 --shape circle --seed 1
  generate_map.py <out prefix> --size 50 --shape blocks --block 4.0 --corridor 1.2
Writes <out prefix>.pgm and <out prefix>.yaml.
"""
import argparse
import os

import numpy as np
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument('out')
parser.add_argument('--size', type=float, default=50.0, help='side of the square map (m)')
parser.add_argument('--resolution', type=float, default=0.05)
parser.add_argument('--density', type=float, default=0.10, help='occupied fraction, 0 to 1')
parser.add_argument('--shape', choices=['circle', 'square', 'u', 'blocks'], default='circle')
parser.add_argument('--seed', type=int, default=1)
parser.add_argument('--block', type=float, default=4.0, help='blocks: side of a block (m)')
parser.add_argument('--corridor', type=float, default=1.2, help='blocks: corridor width (m)')
args = parser.parse_args()

res = args.resolution
n = int(round(args.size / res))
occupied = np.zeros((n, n), dtype=bool)
rng = np.random.default_rng(args.seed)
yy, xx = np.mgrid[0:n, 0:n]


def cells(metres):
    return max(1, int(round(metres / res)))


def stamp_circle():
    r = cells(rng.uniform(0.3, 0.8))
    cx, cy = rng.integers(0, n, 2)
    occupied[(xx - cx) ** 2 + (yy - cy) ** 2 <= r * r] = True


def stamp_square():
    half = cells(rng.uniform(0.3, 0.8))
    cx, cy = rng.integers(0, n, 2)
    occupied[max(0, cy - half):cy + half, max(0, cx - half):cx + half] = True


def stamp_u():
    # A box open on one side: a concave pocket the planner can run into.
    w, h, t = cells(rng.uniform(1.5, 2.5)), cells(rng.uniform(1.5, 2.5)), cells(0.25)
    x0, y0 = rng.integers(0, max(1, n - w)), rng.integers(0, max(1, n - h))
    box = np.zeros((h, w), dtype=bool)
    box[:t, :] = box[-t:, :] = box[:, :t] = box[:, -t:] = True
    side = rng.integers(0, 4)
    if side == 0:
        box[:t, t:-t] = False
    elif side == 1:
        box[-t:, t:-t] = False
    elif side == 2:
        box[t:-t, :t] = False
    else:
        box[t:-t, -t:] = False
    occupied[y0:y0 + h, x0:x0 + w] |= box


if args.shape == 'blocks':
    b, c = cells(args.block), cells(args.corridor)
    period = b + c
    inside = ((xx - c) % period < b) & ((yy - c) % period < b)
    occupied |= inside
else:
    stamp = {'circle': stamp_circle, 'square': stamp_square, 'u': stamp_u}[args.shape]
    while occupied.mean() < args.density:
        stamp()

# A wall around the map, two cells thick.
occupied[:2, :] = occupied[-2:, :] = occupied[:, :2] = occupied[:, -2:] = True

image = np.where(occupied, 0, 254).astype(np.uint8)
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
Image.fromarray(image[::-1]).save(args.out + '.pgm')   # row 0 of the image is the top
with open(args.out + '.yaml', 'w') as f:
    f.write(f'image: {os.path.basename(args.out)}.pgm\nmode: trinary\nresolution: {res}\n'
            'origin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print(f'{args.out}: {n} x {n} cells, {100 * occupied.mean():.1f} % occupied')
