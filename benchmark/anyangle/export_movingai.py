#!/usr/bin/env python3
"""
Convert a blocked grid and its queries (from export_grid.py) to MovingAI .map/.map.scen.

Usage: export_movingai.py <name.grid.txt[.gz]> <name.queries.csv> <out_dir> <name>

The grid file is "W H" followed by H rows of W characters ('1' = blocked), row 0 being
y = 0; queries are grid-vertex coordinates (x, y). The MovingAI map keeps the row order,
so map row = y and map column = x, and the scenario columns are x then y, as the format
defines. The scenario's optimal-length column is not used by R2 and is written as 0.
"""
import csv
import gzip
import os
import sys


def main():
    grid, queries, out, name = sys.argv[1:5]
    opener = gzip.open if grid.endswith('.gz') else open
    with opener(grid, 'rt') as f:
        w, h = (int(v) for v in f.readline().split())
        rows = [f.readline().rstrip('\n') for _ in range(h)]
    assert all(len(r) == w for r in rows), 'row width mismatch'
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, name + '.map'), 'w') as f:
        f.write(f'type octile\nheight {h}\nwidth {w}\nmap\n')
        for r in rows:
            f.write(r.replace('1', '@').replace('0', '.') + '\n')
    with open(queries) as f, open(os.path.join(out, name + '.map.scen'), 'w') as g:
        g.write('version 1\n')
        for q in csv.DictReader(f):
            g.write(f"0\t{name}.map\t{w}\t{h}\t{q['sx']}\t{q['sy']}\t{q['gx']}\t{q['gy']}\t0\n")


if __name__ == '__main__':
    main()
