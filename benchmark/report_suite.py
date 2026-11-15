#!/usr/bin/env python3
"""
Turn the summary.csv of a benchmark suite into tables (Markdown) and plots.

Usage: report_suite.py <out_dir> [title]
Writes <out_dir>/tables.md and, for suites that vary density, size or corridor
width, <out_dir>/plots.png. Prints the tables.
"""
import csv
import os
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

out_dir = sys.argv[1]
title = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(os.path.abspath(out_dir))
rows = list(csv.DictReader(open(os.path.join(out_dir, 'summary.csv'))))
planners = list(dict.fromkeys(r['planner'] for r in rows))
maps = list(dict.fromkeys(r['map'] for r in rows))
cell = {(r['map'], r['planner']): r for r in rows}
# With ground truth from routability.py, count against the pairs that really have
# a route instead of the pairs the reference planner solved cleanly.
truth_file = os.path.join(out_dir, 'summary_truth.csv')
has_truth = os.path.exists(truth_file)
if has_truth:
    for t in csv.DictReader(open(truth_file)):
        r = cell[(t['map'], t['planner'])]
        r['valid'], r['routable'] = t['solved'], t['routable']
        r['touching'] = str(int(t['touching_inscribed']) + int(t['touching_lethal']))
        r['failed'] = t['failed']
lines = [f'# {title}', '']


def num(text, digits=2):
    try:
        value = float(text)
    except ValueError:
        return '-'
    return '-' if value != value else f'{value:.{digits}f}'


def factor(name):
    r = cell[(name, planners[0])]
    if r['group'] == 'density':
        return f"{100 * float(r['density']):g} %"
    if r['group'] == 'size':
        return f"{float(r['size_m']):g} m"
    if r['group'] == 'corridor':
        return f"{float(r['corridor_m']):g} m"
    if r['group'] == 'shape':
        return r['shape']
    return name


groups = list(dict.fromkeys(r['group'] for r in rows))
controlled = bool({'density', 'size', 'corridor', 'shape'} & set(groups))

if controlled:
    labels = {
        'density': 'Obstacle density',
        'size': 'Map size',
        'shape': 'Obstacle shape',
        'corridor': 'Corridor width',
    }
    base = [m for m in maps if m == 'base']
    for group in ('density', 'size', 'shape', 'corridor'):
        members = [m for m in maps if cell[(m, planners[0])]['group'] == group]
        if group in ('size', 'shape'):
            members = sorted(members + base, key=lambda m: float(cell[(m, planners[0])]['size_m']))
        if not members:
            continue
        lines += [
            f'## {labels[group]}',
            '',
            'Collision-free paths out of the pairs that have a route'
            + (' (ground truth from the map)' if has_truth else '')
            + ', and the median planning time.',
            '',
            '| ' + labels[group] + ' | Routable | ' + ' | '.join(planners) + ' |',
            '|---|---|' + '---|' * len(planners),
        ]
        for m in members:
            cells = [
                f"{cell[(m, p)]['valid']} · {num(cell[(m, p)]['time_median_ms'])} ms"
                for p in planners
            ]
            label = 'circle' if (group == 'shape' and m == 'base') else factor(m)
            if group == 'size' and m == 'base':
                label = '50 m'
            lines.append(
                f"| {label} | {cell[(m, planners[0])]['routable']} | " + ' | '.join(cells) + ' |'
            )
        if has_truth:
            lines += [
                '',
                'Routable pairs not solved: paths that touch a cell where the robot '
                'would be in contact with an obstacle, and requests that failed.',
                '',
                '| ' + labels[group] + ' | ' + ' | '.join(planners) + ' |',
                '|---|' + '---|' * len(planners),
            ]
            for m in members:
                cells = [
                    (
                        f"{cell[(m, p)]['touching']} touching, {cell[(m, p)]['failed']} failed"
                        if int(cell[(m, p)]['touching']) + int(cell[(m, p)]['failed'])
                        else '-'
                    )
                    for p in planners
                ]
                label = 'circle' if (group == 'shape' and m == 'base') else factor(m)
                if group == 'size' and m == 'base':
                    label = '50 m'
                lines.append(f'| {label} | ' + ' | '.join(cells) + ' |')
        lines += [
            '',
            'Slowest plan (ms), path length relative to '
            + planners[0]
            + ', and average costmap cost along the path.',
            '',
            '| ' + labels[group] + ' | ' + ' | '.join(planners) + ' |',
            '|---|' + '---|' * len(planners),
        ]
        for m in members:
            cells = [
                ' · '.join(
                    num(cell[(m, p)][key], digits)
                    for key, digits in (
                        ('time_max_ms', 1),
                        ('length_vs_reference', 3),
                        ('mean_cost', 1),
                    )
                )
                for p in planners
            ]
            label = 'circle' if (group == 'shape' and m == 'base') else factor(m)
            if group == 'size' and m == 'base':
                label = '50 m'
            lines.append(f'| {label} | ' + ' | '.join(cells) + ' |')
        lines.append('')

    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    panels = [
        ('density', 'density', lambda r: 100 * float(r['density']), 'Obstacle density (%)'),
        ('size', 'size_m', lambda r: float(r['size_m']), 'Map size (m)'),
        ('corridor', 'corridor_m', lambda r: float(r['corridor_m']), 'Corridor width (m)'),
    ]
    for col, (group, _, x_of, xlabel) in enumerate(panels):
        members = [m for m in maps if cell[(m, planners[0])]['group'] == group]
        if group == 'size':
            members += base
        for p in planners:
            pts = sorted((x_of(cell[(m, p)]), cell[(m, p)]) for m in members)
            xs = [x for x, _ in pts]
            axes[0][col].plot(
                xs, [float(r['time_median_ms']) for _, r in pts], marker='o', label=p
            )
            axes[1][col].plot(
                xs,
                [100.0 * int(r['valid']) / max(1, int(r['routable'])) for _, r in pts],
                marker='o',
                label=p,
            )
        if not members:
            axes[0][col].set_visible(False)
            axes[1][col].set_visible(False)
            continue
        axes[0][col].set_yscale('log')
        axes[0][col].set_ylabel('Median planning time (ms)')
        axes[1][col].set_ylabel('Routable pairs solved (%)')
        for row in (0, 1):
            axes[row][col].set_xlabel(xlabel)
            axes[row][col].grid(True, which='both', alpha=0.3)
        if group == 'size':
            axes[0][col].set_xscale('log')
            axes[1][col].set_xscale('log')
    axes[0][0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'plots.png'), dpi=80)
else:
    for m in maps:
        lines += [
            f'## {m}',
            '',
            '| Variant | Solved | Median ms | 95th pct ms | Slowest ms | Length vs '
            + planners[0]
            + ' | Mean cost | Total turn (rad) |',
            '|---|---|---|---|---|---|---|---|',
        ]
        for p in planners:
            r = cell[(m, p)]
            lines.append(
                f"| {p} | {r['valid']}/{r['routable']} | {num(r['time_median_ms'])} | "
                f"{num(r['time_p95_ms'])} | {num(r['time_max_ms'], 1)} | "
                f"{num(r['length_vs_reference'], 3)} | {num(r['mean_cost'], 1)} | "
                f"{num(r['total_turn_rad'])} |"
            )
        lines.append('')
    lines += [
        '## All maps together',
        '',
        '| Variant | Solved | Mean of median ms | Worst slowest ms | Mean length vs '
        + planners[0]
        + ' | Mean cost |',
        '|---|---|---|---|---|---|',
    ]
    for p in planners:
        mine = [cell[(m, p)] for m in maps]
        ok = [float(r['time_median_ms']) for r in mine if r['time_median_ms'] != 'nan']
        slowest = max(float(r['time_max_ms']) for r in mine if r['time_max_ms'] != 'nan')
        lens = [float(r['length_vs_reference']) for r in mine if r['length_vs_reference'] != 'nan']
        costs = [float(r['mean_cost']) for r in mine if r['mean_cost'] != 'nan']
        lines.append(
            f"| {p} | {sum(int(r['valid']) for r in mine)}/"
            f"{sum(int(r['routable']) for r in mine)} | {sum(ok) / max(1, len(ok)):.2f} | "
            f'{slowest:.1f} | '
            f'{sum(lens) / max(1, len(lens)):.3f} | {sum(costs) / max(1, len(costs)):.1f} |'
        )
    lines.append('')

text = '\n'.join(lines)
with open(os.path.join(out_dir, 'tables.md'), 'w') as f:
    f.write(text + '\n')
print(text)
