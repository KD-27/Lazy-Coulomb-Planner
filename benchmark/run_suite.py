#!/usr/bin/env python3
"""
Run a benchmark suite and collect one summary table.

For each map of the suite: generate it, benchmark every planner on it, and add
its figures to the summary.

A suite lists maps to generate (size, shape, density ...) or existing maps
(`file:` relative to the benchmark directory), and may name its own planners
file (`planners_file:` relative to the suite file) to compare variants of one
planner.

Usage: run_suite.py <suite.yaml> <out_dir>
Writes into <out_dir>: maps/ (generated, with manifest.csv of their checksums),
one CSV per map (every planner on every pair), and summary.csv with one row per
map and planner. A map whose CSV already exists is not run again, so an
interrupted suite can be resumed.
"""
import csv
import hashlib
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml

suite_file, out_dir = sys.argv[1], os.path.abspath(sys.argv[2])
here = os.path.dirname(os.path.abspath(__file__))
suite = yaml.safe_load(open(suite_file))
os.makedirs(os.path.join(out_dir, 'maps'), exist_ok=True)
shutil.copy(suite_file, os.path.join(out_dir, 'suite.yaml'))
planners_file = os.path.join(here, 'config', 'planners.yaml')
if 'planners_file' in suite:
    planners_file = os.path.join(
        os.path.dirname(os.path.abspath(suite_file)), suite['planners_file']
    )
shutil.copy(planners_file, os.path.join(out_dir, 'planners.yaml'))
planners = suite['planners']
# YAML reads bare names such as `off`, `no` or `on` as booleans; insist on strings.
bad = [p for p in planners if not isinstance(p, str)]
if bad:
    sys.exit(f'{suite_file}: planner names must be strings, quote these: {bad}')
env = dict(os.environ, LCP_BENCH_REPEATS=str(suite.get('repeats', 1)))


def median(values):
    return float(np.median(values)) if values else float('nan')


def summarise(spec, rows):
    ref = {r['pair']: r for r in rows if r['planner'] == planners[0]}
    routable = {p for p, r in ref.items() if r['valid'] == '1'}
    out = []
    for planner in planners:
        mine = [r for r in rows if r['planner'] == planner]
        on = [r for r in mine if r['pair'] in routable]
        valid = [r for r in on if r['valid'] == '1']
        times = [float(r['time_ms']) for r in mine if r['returned'] == '1']
        fails = [float(r['wall_ms']) for r in mine if r['returned'] == '0']
        out.append(
            {
                'map': spec['name'],
                'group': spec.get('group', ''),
                'size_m': spec.get('size', ''),
                'shape': spec.get('shape', ''),
                'density': spec.get('density', ''),
                'corridor_m': spec.get('corridor', ''),
                'planner': planner,
                'pairs': len(mine),
                'routable': len(routable),
                'valid': len(valid),
                'colliding': sum(r['returned'] == '1' and r['valid'] == '0' for r in on),
                'failed': sum(r['returned'] == '0' for r in on),
                'returned_without_route': sum(
                    r['returned'] == '1' for r in mine if r['pair'] not in routable
                ),
                'time_median_ms': round(median(times), 3),
                'time_p95_ms': (
                    round(float(np.percentile(times, 95)), 3) if times else float('nan')
                ),
                'time_max_ms': round(max(times), 3) if times else float('nan'),
                'fail_report_max_ms': round(max(fails), 1) if fails else float('nan'),
                'length_vs_reference': round(
                    median([float(r['length']) / float(ref[r['pair']]['length']) for r in valid]),
                    4,
                ),
                'mean_cost': round(median([float(r['mean_cost']) for r in valid]), 2),
                'max_cost': round(median([float(r['max_cost']) for r in valid]), 1),
                'total_turn_rad': round(median([float(r['turn_rad']) for r in valid]), 3),
                'sharp_corner_paths': sum(float(r['sharpest_deg']) > 90.0 for r in valid),
                # No valid path at all (e.g. a planner that always grazes the inscribed
                # zone on a dense map): the median is NaN and cannot be an int.
                'poses_median': (
                    round(median([float(r['poses']) for r in valid])) if valid else float('nan')
                ),
                # Clearance columns exist only in results from 2026-11-17 on.
                'clear_min_m': round(
                    median([float(r['clear_min_m']) for r in valid if 'clear_min_m' in r]), 3),
                'clear_mean_m': round(
                    median([float(r['clear_mean_m']) for r in valid if 'clear_mean_m' in r]), 3),
                'near_frac': round(
                    median([float(r['near_frac']) for r in valid if 'near_frac' in r]), 3),
            }
        )
    return out


summary, manifest = [], []
for index, spec in enumerate(suite['maps'], 1):
    if 'file' in spec:
        # An existing map: use it where it is.
        prefix = os.path.splitext(os.path.join(here, spec['file']))[0]
        args, made = ['file', spec['file']], 'existing map'
    else:
        prefix = os.path.join(out_dir, 'maps', spec['name'])
        args = [
            '--size',
            str(spec['size']),
            '--shape',
            spec['shape'],
            '--seed',
            str(spec.get('seed', 1)),
        ]
        for key in ('density', 'block', 'corridor'):
            if key in spec:
                args += ['--' + key, str(spec[key])]
        made = subprocess.run(
            [sys.executable, os.path.join(here, 'generate_map.py'), prefix] + args,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    digest = hashlib.sha256(open(prefix + '.pgm', 'rb').read()).hexdigest()
    manifest.append(
        {
            'map': spec['name'],
            'arguments': ' '.join(args),
            'sha256': digest,
            'generated': made.split(': ', 1)[-1],
        }
    )

    result = os.path.join(out_dir, spec['name'] + '.csv')
    started = time.time()
    for attempt in (1, 2):
        if os.path.exists(result):
            break
        # The servers of the previous map must be gone before the next ones start.
        for _ in range(30):
            if (
                subprocess.run(
                    ['pgrep', '-f', 'nav2_planner/planner_server'], capture_output=True
                ).returncode
                != 0
            ):
                break
            time.sleep(1)
        subprocess.run(
            [
                os.path.join(here, 'bench.sh'),
                os.path.join(out_dir, 'planners.yaml'),
                prefix + '.yaml',
                result,
                str(suite['pairs']),
            ]
            + planners,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    if not os.path.exists(result):
        print(f'[{index}/{len(suite["maps"])}] {spec["name"]}: FAILED, no result file', flush=True)
        continue
    rows = list(csv.DictReader(open(result)))
    part = summarise(spec, rows)
    summary += part
    brief = ', '.join(
        f"{r['planner']} {r['valid']}/{r['routable']} {r['time_median_ms']:.1f}ms" for r in part
    )
    print(
        f'[{index}/{len(suite["maps"])}] {spec["name"]} ({time.time() - started:.0f} s): {brief}',
        flush=True,
    )

    for name, table in (
        ('summary.csv', summary),
        (os.path.join('maps', 'manifest.csv'), manifest),
    ):
        with open(os.path.join(out_dir, name), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(table[0]))
            w.writeheader()
            w.writerows(table)
print('SUITE DONE', flush=True)
