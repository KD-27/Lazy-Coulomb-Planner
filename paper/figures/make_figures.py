#!/usr/bin/env python3
"""
Figures for the paper, from the v2.1.0 controlled benchmark.

Usage (from the repository root): python3 paper/figures/make_figures.py
Reads experiments/2026-11-15_v2.1.0/controlled{,_seeds}/summary*.csv and writes
PDF figures next to this script.
"""
import csv
import os
import statistics

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

here = os.path.dirname(os.path.abspath(__file__))
data = os.path.join(here, '..', '..', 'experiments', '2026-11-15_v2.1.0')
suites = [os.path.join(data, 'controlled'), os.path.join(data, 'controlled_seeds')]
PLANNERS = [
    ('LCP', 'LCP', 'tab:red', 'o'),
    ('LCP_lazy', 'LCP, lazy search only', 'tab:pink', 'v'),
    ('Navfn', 'NavFn', 'tab:blue', 's'),
    ('Smac2d', 'Smac 2D', 'tab:orange', '^'),
    ('ThetaStar', 'Theta*', 'tab:green', 'D'),
]
plt.rcParams.update(
    {
        'font.size': 8,
        'font.family': 'serif',
        'axes.linewidth': 0.6,
        'lines.linewidth': 1.0,
        'lines.markersize': 3.5,
        'legend.fontsize': 7,
    }
)

rows = {}  # (shape, size, density) -> list of {planner: (time, clean %)}
for d in suites:
    truth = {
        (r['map'], r['planner']): r
        for r in csv.DictReader(open(os.path.join(d, 'summary_truth.csv')))
    }
    by_map = {}
    for r in csv.DictReader(open(os.path.join(d, 'summary.csv'))):
        by_map.setdefault(r['map'], {})[r['planner']] = r
    for name, entries in by_map.items():
        lcp = entries['LCP']
        if lcp['corridor_m'] or lcp['shape'] not in ('circle',):
            continue
        key = (float(lcp['size_m']), float(lcp['density']))
        sample = {}
        for p in entries:
            t = truth[(name, p)]
            sample[p] = (
                float(entries[p]['time_median_ms']),
                100.0 * int(t['solved']) / max(1, int(t['routable'])),
            )
        rows.setdefault(key, []).append(sample)


def series(keys, planner, index):
    xs, means, sds = [], [], []
    for x, key in keys:
        values = [s[planner][index] for s in rows[key]]
        xs.append(x)
        means.append(statistics.mean(values))
        sds.append(statistics.stdev(values) if len(values) > 1 else 0.0)
    return xs, means, sds


density = sorted(((100 * k[1], k) for k in rows if k[0] == 50.0))
size = sorted(((k[0], k) for k in rows if k[1] == 0.10))


def figure(keys, index, xlabel, ylabel, name, logx=False, logy=False, planners=PLANNERS):
    fig, ax = plt.subplots(figsize=(3.4, 2.3))
    for p, label, colour, marker in planners:
        xs, m, sd = series(keys, p, index)
        ax.errorbar(xs, m, yerr=sd, label=label, color=colour, marker=marker, capsize=2)
    if logx:
        ax.set_xscale('log')
    if logy:
        ax.set_yscale('log')
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, which='both', alpha=0.25, linewidth=0.4)
    ax.legend(frameon=False)
    fig.tight_layout(pad=0.3)
    fig.savefig(os.path.join(here, name))
    print('wrote', name)


figure(
    density,
    0,
    'Obstacle density (%)',
    'Median planning time (ms)',
    'time_vs_density.pdf',
    logy=True,
)
figure(
    size, 0, 'Map side (m)', 'Median planning time (ms)', 'time_vs_size.pdf', logx=True, logy=True
)
figure(
    density, 1, 'Obstacle density (%)', 'Routable pairs solved cleanly (%)', 'clean_vs_density.pdf'
)

# Speed-up of LCP over the fastest grid planner, per map, then mean and sd.
fig, ax = plt.subplots(figsize=(3.4, 2.1))
for keys, label, colour in ((density, 'vs. obstacle density (50 m map)', 'tab:red'),):
    xs, means, sds = [], [], []
    for x, key in keys:
        ratios = [
            min(s[p][0] for p in ('Navfn', 'Smac2d', 'ThetaStar')) / s['LCP'][0] for s in rows[key]
        ]
        xs.append(x)
        means.append(statistics.mean(ratios))
        sds.append(statistics.stdev(ratios) if len(ratios) > 1 else 0.0)
    ax.errorbar(xs, means, yerr=sds, color=colour, marker='o', capsize=2)
ax.axhline(1.0, color='black', linewidth=0.6, linestyle='--')
ax.set_yscale('log')
ax.set_xlabel('Obstacle density (%)')
ax.set_ylabel('Speed-up over fastest\ngrid planner')
ax.grid(True, which='both', alpha=0.25, linewidth=0.4)
fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(here, 'speedup_vs_density.pdf'))
print('wrote speedup_vs_density.pdf')
