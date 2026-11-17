#!/usr/bin/env python3
"""
Compare LCP driving runs with path reuse on and off.

Reads <batch>/reuse_{on,off}/runs.csv (written by sim/summarise_runs.py) and the original
2026-11-15 runs for the same controller, and prints means, standard deviations and
two-sided Mann-Whitney p-values.

Usage: python3 compare_reuse.py <batch dir> <original driving dir> [<label>]
"""
import csv
import os
import statistics
import sys

from scipy.stats import mannwhitneyu

batch, original = sys.argv[1], sys.argv[2]
sets = {
    'reuse off (new)': os.path.join(batch, 'reuse_off', 'runs.csv'),
    'reuse on (new)': os.path.join(batch, 'reuse_on', 'runs.csv'),
    'reuse on (2026-11-15)': os.path.join(original, 'runs.csv'),
}
data = {}
for name, path in sets.items():
    rows = [r for r in csv.DictReader(open(path)) if r['planner'] == 'lcp']
    data[name] = rows
keys = ['reached', 'recoveries', 'sim_time_s', 'travelled_m', 'closest_obstacle_m']


def ms(v):
    """Format mean and standard deviation."""
    return f'{statistics.mean(v):.1f} ± {statistics.stdev(v):.1f}' if len(v) > 1 else f'{v[0]:.1f}'


print(f'# {sys.argv[3] if len(sys.argv) > 3 else os.path.basename(batch)}\n')
print('| Set | Runs | ' + ' | '.join(keys) + ' |')
print('|---|---|' + '---|' * len(keys))
for name, rows in data.items():
    print(f'| {name} | {len(rows)} | ' +
          ' | '.join(ms([float(r[k]) for r in rows]) for k in keys) + ' |')
print('\nMann-Whitney U, two-sided, reuse off (new) against:\n')
print('| Against | ' + ' | '.join(keys) + ' |')
print('|---|' + '---|' * len(keys))
off = data['reuse off (new)']
for name in ['reuse on (new)', 'reuse on (2026-11-15)']:
    ps = []
    for k in keys:
        a = [float(r[k]) for r in off]
        b = [float(r[k]) for r in data[name]]
        ps.append('n/a' if len(set(a + b)) == 1 else f'{mannwhitneyu(a, b).pvalue:.3g}')
    print(f'| {name} | ' + ' | '.join(ps) + ' |')
