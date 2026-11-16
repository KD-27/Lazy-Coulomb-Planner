#!/usr/bin/env python3
"""
Generate an occupancy map with controlled obstacle density, shape and size.

Random maps place obstacles of one shape until the requested fraction of the map
is occupied. `blocks` maps are a regular grid of square blocks separated by
corridors of a fixed width. The same arguments and seed always give the same map.

Usage:
  generate_map.py <out prefix> --size 50 --density 0.10 --shape circle --seed 1
  generate_map.py <out prefix> --size 50 --shape blocks --block 4.0 --corridor 1.2
Writes <out prefix>.pgm and <out prefix>.yaml. With --sdf it also writes
<out prefix>.sdf, a Gazebo world with the same obstacles (0.5 m tall) for the
driving test: circles become cylinders, everything else boxes that cover
exactly the occupied cells. The map image does not depend on --sdf.
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
parser.add_argument('--sdf', action='store_true', help='also write a Gazebo world')
args = parser.parse_args()

res = args.resolution
n = int(round(args.size / res))
occupied = np.zeros((n, n), dtype=bool)
# For the Gazebo world: circles as (centre cell x, y, radius in cells), and every
# other obstacle as cells in a separate mask that is later cut into boxes.
circles = []
box_cells = np.zeros((n, n), dtype=bool)
rng = np.random.default_rng(args.seed)
yy, xx = np.mgrid[0:n, 0:n]


def cells(metres):
    return max(1, int(round(metres / res)))


def fill(y0, y1, x0, x1, mask):
    """Mark `mask` inside the window as occupied; return how many cells that added."""
    window = occupied[y0:y1, x0:x1]
    before = int(window.sum())
    window |= mask
    return int(window.sum()) - before


def stamp_circle():
    r = cells(rng.uniform(0.3, 0.8))
    cx, cy = rng.integers(0, n, 2)
    y0, y1, x0, x1 = max(0, cy - r), min(n, cy + r + 1), max(0, cx - r), min(n, cx + r + 1)
    inside = (xx[y0:y1, x0:x1] - cx) ** 2 + (yy[y0:y1, x0:x1] - cy) ** 2 <= r * r
    circles.append((int(cx), int(cy), int(r)))
    return fill(y0, y1, x0, x1, inside)


def stamp_square():
    half = cells(rng.uniform(0.3, 0.8))
    cx, cy = rng.integers(0, n, 2)
    box_cells[max(0, cy - half):cy + half, max(0, cx - half):cx + half] = True
    return fill(max(0, cy - half), cy + half, max(0, cx - half), cx + half, True)


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
    box_cells[y0:y0 + h, x0:x0 + w] |= box[:box_cells[y0:y0 + h, x0:x0 + w].shape[0],
                                           :box_cells[y0:y0 + h, x0:x0 + w].shape[1]]
    return fill(y0, y0 + h, x0, x0 + w, box)


if args.shape == 'blocks':
    b, c = cells(args.block), cells(args.corridor)
    period = b + c
    inside = ((xx - c) % period < b) & ((yy - c) % period < b)
    occupied |= inside
else:
    # Count occupied cells as they are added: recomputing the mean of a large map
    # after every obstacle is what made big maps slow to generate.
    stamp = {'circle': stamp_circle, 'square': stamp_square, 'u': stamp_u}[args.shape]
    count = 0
    while count / (n * n) < args.density:
        count += stamp()

# A wall around the map, two cells thick.
occupied[:2, :] = occupied[-2:, :] = occupied[:, :2] = occupied[:, -2:] = True
if args.shape == 'blocks':
    box_cells |= occupied
box_cells[:2, :] = box_cells[-2:, :] = box_cells[:, :2] = box_cells[:, -2:] = True

image = np.where(occupied, 0, 254).astype(np.uint8)
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
Image.fromarray(image[::-1]).save(args.out + '.pgm')   # row 0 of the image is the top
with open(args.out + '.yaml', 'w') as f:
    f.write(f'image: {os.path.basename(args.out)}.pgm\nmode: trinary\nresolution: {res}\n'
            'origin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print(f'{args.out}: {n} x {n} cells, {100 * occupied.mean():.1f} % occupied')


def rectangles(mask):
    """
    Cover the True cells of `mask` exactly with axis-aligned rectangles.

    A greedy merge: runs along rows, merged with identical runs in the rows
    above. Returns (x0, y0, x1, y1) in cells, end exclusive.
    """
    done = []
    open_runs = {}   # (x0, x1) -> starting row
    for y in range(mask.shape[0] + 1):
        runs = set()
        if y < mask.shape[0]:
            row = np.concatenate(([False], mask[y], [False]))
            edges = np.flatnonzero(row[1:] != row[:-1])
            runs = {(int(a), int(b)) for a, b in zip(edges[::2], edges[1::2])}
        for run in list(open_runs):
            if run not in runs:
                done.append((run[0], open_runs.pop(run), run[1], y))
        for run in runs:
            open_runs.setdefault(run, y)
    return done


if args.sdf:
    height = 0.5
    parts = []
    for i, (cx, cy, r) in enumerate(circles):
        # A cell is occupied when its centre is within r cells of the circle's centre
        # cell, so a cylinder of radius r cells contains exactly those centres. The
        # cylinder is up to half a cell smaller than the occupied cells: the map
        # errs on the safe side.
        x, y, radius = (cx + 0.5) * res, (cy + 0.5) * res, r * res + 1e-4
        shape = f'<cylinder><radius>{radius:.4f}</radius><length>{height}</length></cylinder>'
        parts.append((f'c{i}', x, y, shape))
    for i, (x0, y0, x1, y1) in enumerate(rectangles(box_cells)):
        w, h = (x1 - x0) * res, (y1 - y0) * res
        shape = f'<box><size>{w:.4f} {h:.4f} {height}</size></box>'
        parts.append((f'b{i}', (x0 + x1) / 2 * res, (y0 + y1) / 2 * res, shape))
    elements = []
    for name, x, y, shape in parts:
        pose = f'<pose>{x:.4f} {y:.4f} {height / 2} 0 0 0</pose>'
        elements.append(
            f'        <collision name="{name}">{pose}<geometry>{shape}</geometry></collision>\n'
            f'        <visual name="{name}">{pose}<geometry>{shape}</geometry>'
            '<material><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>')
    template = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'world_template.sdf')).read()
    world = template.replace('<!-- OBSTACLES -->', '\n'.join(elements))
    world = world.replace('GROUND_SIZE', f'{2 * args.size + 20:g}')
    with open(args.out + '.sdf', 'w') as f:
        f.write(world)
    print(f'{args.out}.sdf: {len(circles)} cylinders, {len(parts) - len(circles)} boxes')
