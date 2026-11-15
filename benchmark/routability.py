#!/usr/bin/env python3
"""
Ground truth for a benchmark suite: which start/goal pairs really have a route.

The benchmark takes its reference planner's word for whether a route exists.
This recomputes it from the map alone: cells within the robot's inscribed
radius of an obstacle are blocked, the rest are grouped into connected regions
(moves between cells that share an edge), and a pair has a route when start and
goal lie in the same region. It then re-scores every planner against that.

The blocked radius defaults to 0.224 m for the benchmark's 0.22 m robot: Nav2's
inflation layer also marks cells whose centre is 4.47 cells (0.2236 m) from an
obstacle cell as inscribed. With that value the result agrees with what NavFn
and LCP's own grid search conclude from the live costmap on every pair; with
exactly 0.22 m six pairs on the densest map differ.

Usage: routability.py <suite out_dir> [blocked radius, default 0.224]
Writes <out_dir>/summary_truth.csv and prints a table.
"""
import csv
import os
import sys

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

out_dir = os.path.abspath(sys.argv[1])
radius = float(sys.argv[2]) if len(sys.argv) > 2 else 0.224
here = os.path.dirname(os.path.abspath(__file__))
suite = yaml.safe_load(open(os.path.join(out_dir, 'suite.yaml')))
planners = suite['planners']

rows_out = []
for spec in suite['maps']:
    result = os.path.join(out_dir, spec['name'] + '.csv')
    if not os.path.exists(result):
        continue
    prefix = (os.path.splitext(os.path.join(here, spec['file']))[0] if 'file' in spec
              else os.path.join(out_dir, 'maps', spec['name']))
    meta = yaml.safe_load(open(prefix + '.yaml'))
    img = np.array(Image.open(prefix + '.pgm'))[::-1]          # row 0 = lowest y
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    dark = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    not_free = dark >= meta['free_thresh']                     # occupied or unknown
    # Same rule as Nav2's inflation layer: blocked if the cell centre is within the
    # inscribed radius of an occupied cell.
    blocked = ndimage.distance_transform_edt(~not_free) * res <= radius
    region, _ = ndimage.label(~blocked)                        # 4-connected by default

    def region_of(x, y):
        c, r = int((x - ox) / res), int((y - oy) / res)
        inside = 0 <= r < region.shape[0] and 0 <= c < region.shape[1]
        return int(region[r, c]) if inside else 0

    rows = list(csv.DictReader(open(result)))
    truth = {}
    for r in rows:
        a = region_of(float(r['sx']), float(r['sy']))
        b = region_of(float(r['gx']), float(r['gy']))
        truth[r['pair']] = a != 0 and a == b
    routable = sum(truth.values())
    for planner in planners:
        mine = [r for r in rows if r['planner'] == planner]
        on = [r for r in mine if truth[r['pair']]]
        off = [r for r in mine if not truth[r['pair']]]
        rows_out.append({
            'map': spec['name'], 'planner': planner, 'pairs': len(mine), 'routable': routable,
            'solved': sum(r['valid'] == '1' for r in on),
            'touching_inscribed': sum(r['returned'] == '1' and r['max_cost'] == '253' for r in on),
            'touching_lethal': sum(r['returned'] == '1' and int(r['max_cost']) >= 254 for r in on),
            'failed': sum(r['returned'] == '0' for r in on),
            'clean_path_without_route': sum(r['valid'] == '1' for r in off),
            'bad_path_without_route': sum(r['returned'] == '1' and r['valid'] == '0' for r in off),
            'correctly_refused': sum(r['returned'] == '0' for r in off),
        })

with open(os.path.join(out_dir, 'summary_truth.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows_out[0]))
    w.writeheader()
    w.writerows(rows_out)

print(f"{'map':>14} {'routable':>8} | " + ' | '.join(f'{p:>22}' for p in planners))
print(' ' * 25 + ' | '.join(f"{'solved insc leth fail':>22}" for _ in planners))
for name in dict.fromkeys(r['map'] for r in rows_out):
    cells = []
    for p in planners:
        r = next(x for x in rows_out if x['map'] == name and x['planner'] == p)
        cells.append(f"{r['solved']:>6} {r['touching_inscribed']:>4} {r['touching_lethal']:>4} "
                     f"{r['failed']:>4}  ")
    print(f"{name:>14} {rows_out[[x['map'] for x in rows_out].index(name)]['routable']:>8} | "
          + ' | '.join(cells))
print('\nPairs with no route: clean path returned / bad path returned / correctly refused')
for p in planners:
    mine = [x for x in rows_out if x['planner'] == p]
    print(f"{p:>10}: {sum(x['clean_path_without_route'] for x in mine)} / "
          f"{sum(x['bad_path_without_route'] for x in mine)} / "
          f"{sum(x['correctly_refused'] for x in mine)}")
