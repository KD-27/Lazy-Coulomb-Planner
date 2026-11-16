#!/usr/bin/env python3
"""
Summarise repeated driving-test runs.

Usage: summarise_runs.py <dir> [reference planner, default lcp]
<dir> holds one sub-directory per run named <planner>_<nn>, each with
<planner>.csv from sim/drive.py. Prints a Markdown table (one row per planner)
and two-sided Mann-Whitney U tests of the reference planner against each other
planner, and writes runs.csv with one row per run.
"""
import csv
import glob
import os
import statistics
import sys

from scipy.stats import mannwhitneyu

root = sys.argv[1]
reference = sys.argv[2] if len(sys.argv) > 2 else 'lcp'
runs = []
for path in sorted(glob.glob(os.path.join(root, '*_[0-9][0-9]', '*.csv'))):
    planner = os.path.basename(path)[:-4]
    legs = list(csv.DictReader(open(path)))
    runs.append(
        {
            'run': os.path.basename(os.path.dirname(path)),
            'planner': planner,
            'goals': len(legs),
            'reached': sum(int(x['reached']) for x in legs),
            'sim_time_s': round(sum(float(x['sim_time_s']) for x in legs), 1),
            'travelled_m': round(sum(float(x['travelled_m']) for x in legs), 2),
            'recoveries': sum(int(x['recoveries']) for x in legs),
            'closest_obstacle_m': min(float(x['min_scan_range_m']) for x in legs),
        }
    )
with open(os.path.join(root, 'runs.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(runs[0]))
    w.writeheader()
    w.writerows(runs)

planners = list(dict.fromkeys(r['planner'] for r in runs))


def spread(values):
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return f'{statistics.mean(values):.1f} ± {sd:.1f}'


goal_counts = sorted({r['goals'] for r in runs})
goals_label = '/'.join(str(g) for g in goal_counts)
print(
    f'| Planner | Runs | Goals reached (of {goals_label}), mean ± sd | Runs with all goals | '
    'Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |'
)
print('|---|---|---|---|---|---|---|---|---|')
for p in planners:
    mine = [r for r in runs if r['planner'] == p]
    print(
        f"| {p} | {len(mine)} | {spread([r['reached'] for r in mine])} | "
        f"{sum(r['reached'] == r['goals'] for r in mine)} | "
        f"{spread([r['sim_time_s'] for r in mine])} | {spread([r['recoveries'] for r in mine])} | "
        f"{statistics.median([r['recoveries'] for r in mine]):g} | "
        f"{spread([r['travelled_m'] for r in mine])} | "
        f"{statistics.mean([r['closest_obstacle_m'] for r in mine]):.2f} |"
    )

print(f'\nMann-Whitney U, two-sided, {reference} against each other planner (p-values):\n')
print('| Against | Goals reached | Recoveries | Sim time |')
print('|---|---|---|---|')
ref = [r for r in runs if r['planner'] == reference]
for p in planners:
    if p == reference:
        continue
    other = [r for r in runs if r['planner'] == p]
    cells = []
    for key in ('reached', 'recoveries', 'sim_time_s'):
        a, b = [r[key] for r in ref], [r[key] for r in other]
        try:
            cells.append(f'{mannwhitneyu(a, b, alternative="two-sided").pvalue:.3g}')
        except ValueError:
            cells.append('identical')
    print(f'| {p} | ' + ' | '.join(cells) + ' |')
