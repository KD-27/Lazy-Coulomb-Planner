#!/usr/bin/env python3
"""
Combine suites that repeat the same map configurations with different seeds.

Usage: aggregate_seeds.py <out prefix> <suite dir> [<suite dir> ...]
Every suite dir needs summary.csv and summary_truth.csv (routability.py). Maps
are grouped by (group, size, shape, density); each group's figures are the mean
and standard deviation over its maps. Writes <out prefix>.md and <out prefix>.png.
"""
import csv
import os
import statistics
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

prefix, dirs = sys.argv[1], sys.argv[2:]
GRID = ('Navfn', 'Smac2d', 'ThetaStar')
data = {}  # (group, size, shape, density) -> list of per-map dicts
for d in dirs:
    truth = {
        (r['map'], r['planner']): r
        for r in csv.DictReader(open(os.path.join(d, 'summary_truth.csv')))
    }
    rows = list(csv.DictReader(open(os.path.join(d, 'summary.csv'))))
    for name in dict.fromkeys(r['map'] for r in rows):
        mine = {r['planner']: r for r in rows if r['map'] == name}
        r0 = mine['LCP']
        if r0['shape'] not in ('circle', 'u') or r0['corridor_m']:
            continue
        key = (r0['shape'], float(r0['size_m']), float(r0['density']))
        routable = int(truth[(name, 'LCP')]['routable'])
        entry = {'routable': routable}
        for p in mine:
            entry[p + '_ms'] = float(mine[p]['time_median_ms'])
            entry[p + '_solved'] = 100.0 * int(truth[(name, p)]['solved']) / max(1, routable)
        entry['grid_ms'] = min(entry[p + '_ms'] for p in GRID)
        entry['speedup'] = entry['grid_ms'] / entry['LCP_ms']
        data.setdefault(key, []).append(entry)


def ms(values, digits=2):
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return f'{statistics.mean(values):.{digits}f} ± {sd:.{digits}f}'


def table(title, keys, label):
    out = [
        f'## {title}',
        '',
        '| ' + label + ' | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | '
        'LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | '
        'Theta* clean (%) |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for key, name in keys:
        e = data[key]
        out.append(
            f"| {name} | {len(e)} | {ms([x['LCP_ms'] for x in e])} | "
            f"{ms([x['grid_ms'] for x in e])} | {ms([x['speedup'] for x in e], 1)}× | "
            + ' | '.join(
                ms([x[p + '_solved'] for x in e], 1)
                for p in ('LCP', 'LCP_lazy', 'Navfn', 'Smac2d', 'ThetaStar')
            )
            + ' |'
        )
    return out + ['']


density = sorted(k for k in data if k[0] == 'circle' and k[1] == 50.0)
size = sorted(k for k in data if k[0] == 'circle' and k[2] == 0.10)
ushape = sorted(k for k in data if k[0] == 'u')
lines = [
    '# Controlled benchmark over several random seeds',
    '',
    'Mean ± standard deviation over the maps generated with different seeds.',
    '',
]
lines += table(
    'Obstacle density (50 m map, random circles)',
    [(k, f'{100 * k[2]:g} %') for k in density],
    'Density',
)
lines += table('Map size (10 % density)', [(k, f'{k[1]:g} m') for k in size], 'Size')
if ushape:
    lines += table(
        'Concave U-shaped obstacles (50 m, 10 %)', [(k, 'U shapes') for k in ushape], 'Shape'
    )
open(prefix + '.md', 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines))

fig, axes = plt.subplots(1, 3, figsize=(18, 5))
xs = [100 * k[2] for k in density]
for name, colour in (
    ('LCP', 'tab:red'),
    ('LCP_lazy', 'tab:purple'),
    ('Navfn', 'tab:blue'),
    ('Smac2d', 'tab:orange'),
    ('ThetaStar', 'tab:green'),
):
    mean = [statistics.mean(x[name + '_ms'] for x in data[k]) for k in density]
    sd = [
        statistics.stdev([x[name + '_ms'] for x in data[k]]) if len(data[k]) > 1 else 0
        for k in density
    ]
    axes[0].errorbar(xs, mean, yerr=sd, marker='o', capsize=3, label=name, color=colour)
    solved = [statistics.mean(x[name + '_solved'] for x in data[k]) for k in density]
    sd2 = [
        statistics.stdev([x[name + '_solved'] for x in data[k]]) if len(data[k]) > 1 else 0
        for k in density
    ]
    axes[1].errorbar(xs, solved, yerr=sd2, marker='o', capsize=3, label=name, color=colour)
    sx = [k[1] for k in size]
    mean3 = [statistics.mean(x[name + '_ms'] for x in data[k]) for k in size]
    sd3 = [
        statistics.stdev([x[name + '_ms'] for x in data[k]]) if len(data[k]) > 1 else 0
        for k in size
    ]
    axes[2].errorbar(sx, mean3, yerr=sd3, marker='o', capsize=3, label=name, color=colour)
axes[0].set_yscale('log')
axes[0].set_xlabel('Obstacle density (%)')
axes[0].set_ylabel('Median planning time (ms)')
axes[1].set_xlabel('Obstacle density (%)')
axes[1].set_ylabel('Routable pairs solved with a clean path (%)')
axes[2].set_xscale('log')
axes[2].set_yscale('log')
axes[2].set_xlabel('Map size (m)')
axes[2].set_ylabel('Median planning time (ms)')
for ax in axes:
    ax.grid(True, which='both', alpha=0.3)
axes[0].legend()
fig.suptitle('Controlled benchmark, mean ± sd over seeds')
fig.tight_layout()
fig.savefig(prefix + '.png', dpi=80)
