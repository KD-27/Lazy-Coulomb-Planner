#!/usr/bin/env python3
"""
Compare the Java any-angle baselines with the planners of a benchmark suite.

Usage: compare.py <suite_dir> <anyangle_dir> [resolution]
Writes <anyangle_dir>/comparison.csv and comparison.md: per map and planner, the
pairs solved, median and 95th-percentile planning time, and the median path
length relative to Anya (optimal any-angle on the same blocked grid). Only pairs
that Anya solves are counted for the length ratio.
Times of the Java planners are measured in Java (median of repeats after a
warm-up); times of the Nav2 planners are planner_server's own planning time.
"""
import csv
import os
import sys

import numpy as np

suite, aa_dir = sys.argv[1], sys.argv[2]
res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
ORDER = ['LCP', 'LCP_lazy', 'Navfn', 'Smac2d', 'ThetaStar',
         'Anya16', 'JPS', 'JPS_PS', 'Theta', 'LazyTheta']


def med(v):
    return float(np.median(v)) if v else float('nan')


def p95(v):
    return float(np.percentile(v, 95)) if v else float('nan')


rows = []
for f in sorted(os.listdir(aa_dir)):
    if not f.endswith('.csv') or f.startswith('comparison'):
        continue
    name = f[:-4]
    bench = os.path.join(suite, name + '.csv')
    if not os.path.exists(bench):
        continue
    per = {}  # planner -> pair -> (ok, time, length_m)
    for r in csv.DictReader(open(bench)):
        ok = r['valid'] == '1'
        per.setdefault(r['planner'], {})[int(r['pair'])] = (
            ok, float(r['time_ms']), float(r['length']) if ok else float('nan'))
    for r in csv.DictReader(open(os.path.join(aa_dir, f))):
        ok = r['found'] == '1'
        per.setdefault(r['algo'], {})[int(r['pair'])] = (
            ok, float(r['time_ms']), float(r['length_cells']) * res if ok else float('nan'))
    anya = per.get('Anya16', {})
    for planner in [p for p in ORDER if p in per] + sorted(set(per) - set(ORDER)):
        d = per[planner]
        solved = [p for p, v in d.items() if v[0]]
        times = [d[p][1] for p in solved]
        ratio = [d[p][2] / anya[p][2] for p in solved
                 if p in anya and anya[p][0] and anya[p][2] > 0]
        rows.append({'map': name, 'planner': planner, 'pairs': len(d),
                     'solved': len(solved), 'anya_solved': sum(v[0] for v in anya.values()),
                     'time_median_ms': round(med(times), 3), 'time_p95_ms': round(p95(times), 3),
                     'length_vs_anya': round(med(ratio), 4),
                     'length_vs_anya_p95': round(p95(ratio), 4)})

with open(os.path.join(aa_dir, 'comparison.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

planners = [p for p in ORDER if any(r['planner'] == p for r in rows)]
with open(os.path.join(aa_dir, 'comparison.md'), 'w') as f:
    for title, key, fmt in (('Median planning time (ms)', 'time_median_ms', '{:.2f}'),
                            ('Median path length / Anya', 'length_vs_anya', '{:.3f}'),
                            ('Pairs solved', 'solved', '{}')):
        f.write(f'### {title}\n\n| map | ' + ' | '.join(planners) + ' |\n')
        f.write('|---' * (len(planners) + 1) + '|\n')
        for m in dict.fromkeys(r['map'] for r in rows):
            cells = []
            for p in planners:
                r = next((r for r in rows if r['map'] == m and r['planner'] == p), None)
                cells.append(fmt.format(r[key]) if r else '')
            f.write(f'| {m} | ' + ' | '.join(cells) + ' |\n')
        f.write('\n')
print(open(os.path.join(aa_dir, 'comparison.md')).read())
