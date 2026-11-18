#!/usr/bin/env python3
"""
Summarise R2 results against Anya (optimal) and the planners in comparison.csv.

Usage: compare_r2.py <anyangle_dir> <r2_out_dir>
Writes <r2_out_dir>/r2_summary.csv and prints a markdown table. Per pair, the time is
the median of the repeats; per map, the median and 95th percentile over pairs.
"""
import csv
import glob
import math
import os
import statistics
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(math.ceil(q * len(v))) - 1)] if v else float('nan')


def r2_map(res_dir, name):
    runs = sorted(glob.glob(os.path.join(res_dir, name + '.R2.*.results')))
    times, paths = None, None
    for r in runs:
        rows = [ln.split('\t') for ln in open(r).read().splitlines()]
        if times is None:
            times = [[] for _ in rows]
            paths = [list(map(float, x[1:])) if x[0] != 'timeout' else [] for x in rows]
        for i, x in enumerate(rows):
            if x[0] != 'timeout':   # query dropped by run_r2_safe.py (R2 did not return)
                times[i].append(float(x[0]) / 1e6)
    lengths = []
    for p in paths:
        pts = list(zip(p[0::2], p[1::2]))
        lengths.append(sum(math.dist(pts[k], pts[k + 1]) for k in range(len(pts) - 1))
                       if len(pts) > 1 else None)
    timeouts = sum(1 for t in times if not t)
    return [statistics.median(t) for t in times if t], lengths, len(runs), timeouts


def main():
    aa, out = sys.argv[1:3]
    comp = {(r['map'], r['planner']): r
            for r in csv.DictReader(open(os.path.join(aa, 'comparison.csv')))}
    maps = sorted({m for m, _ in comp})
    rows = []
    for m in maps:
        anya = {int(r['pair']): r for r in csv.DictReader(open(os.path.join(aa, m + '.csv')))
                if r['algo'] == 'Anya16'}
        t, lens, nrep, nto = r2_map(os.path.join(out, 'results'), m.replace('.', 'p'))
        solved = sum(1 for x in lens if x is not None)
        anya_ok = [anya[i]['found'] == '1' for i in range(len(lens))]
        both = [lens[i] / float(anya[i]['length_cells']) for i in range(len(lens))
                if lens[i] is not None and anya_ok[i] and float(anya[i]['length_cells']) > 0]
        mismatch = sum(1 for i in range(len(lens)) if (lens[i] is not None) != anya_ok[i])
        rows.append({
            'map': m, 'repeats': nrep, 'pairs': len(lens), 'r2_solved': solved, 'r2_timeouts': nto,
            'anya_solved': sum(1 for a in anya.values() if a['found'] == '1'),
            'solved_mismatch_vs_anya': mismatch,
            'r2_time_median_ms': round(statistics.median(t), 4),
            'r2_time_p95_ms': round(pct(t, 0.95), 4),
            'r2_length_vs_anya_median': round(statistics.median(both), 5) if both else '',
            'r2_length_vs_anya_max': round(max(both), 5) if both else '',
            **{f'{p}_time_median_ms': comp[(m, p)]['time_median_ms']
               for p in ('LCP', 'Anya16', 'JPS_PS', 'Navfn') if (m, p) in comp}})
    with open(os.path.join(out, 'r2_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('| map | R2 | LCP | Anya | JPS+PS | R2 solved / Anya | R2 len / Anya (median, max) |')
    print('|---|---|---|---|---|---|---|')
    for r in rows:
        print(f"| {r['map']} | {r['r2_time_median_ms']} | {r.get('LCP_time_median_ms')} | "
              f"{r.get('Anya16_time_median_ms')} | "
              f"{r.get('JPS_PS_time_median_ms')} | {r['r2_solved']} / {r['anya_solved']} | "
              f"{r['r2_length_vs_anya_median']}, {r['r2_length_vs_anya_max']} |")


if __name__ == '__main__':
    main()
