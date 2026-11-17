#!/usr/bin/env python3
"""
Offline analyses for the review, from data recorded on 2026-11-15 (v2.1.0).

1. lazy_vs_fallback: on the 69 controlled maps, LCP's path quality on the requests the
   lazy correction solves by itself (LCP_lazy valid) against those that need the
   weighted A* fallback, with NavFn on the same pairs to show how hard each group is.
2. side_rule: per-pair comparison of the Coulomb side rule (r2, default) with the
   nearest-free-side probe on the eight ablation maps, including the pairs one of
   them solves lazily and the other needs the fallback for.
3. tails: median, p95 and max planning time per planner and density (50 m maps,
   pooled over seeds) for the paper's Table I.

Usage: python3 analyse.py <experiments/2026-11-15_v2.1.0> <out dir>
"""
import glob
import os
import sys

import pandas as pd

src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
report = []


def load(dirs):
    """Read every per-map CSV of the given result directories."""
    frames = []
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(src, d, '*.csv'))):
            name = os.path.basename(f)[:-4]
            if name.startswith('summary'):
                continue
            df = pd.read_csv(f)
            df['map'] = name
            frames.append(df)
    return pd.concat(frames, ignore_index=True)


def wide(df, metrics):
    """Pivot to one row per (map, pair) and one column per (metric, planner)."""
    return df.pivot_table(index=['map', 'pair'], columns='planner', values=metrics)


# ---------------------------------------------------------------- 1. lazy vs fallback
ctl = load(['controlled', 'controlled_seeds'])
w = wide(ctl, ['valid', 'length', 'mean_cost', 'max_cost', 'turn_rad', 'time_ms', 'straight'])
ok = w[('valid', 'LCP')] == 1
lazy = ok & (w[('valid', 'LCP_lazy')] == 1)
fb = ok & ~(w[('valid', 'LCP_lazy')] == 1)
both_nav = w[('valid', 'Navfn')] == 1
rows = []
for name, m in [('lazy-solved', lazy), ('fallback-solved', fb)]:
    sub = w[m]
    subn = w[m & both_nav]
    rows.append({
        'group': name,
        'requests': int(m.sum()),
        'LCP time median ms': sub[('time_ms', 'LCP')].median(),
        'LCP time p95 ms': sub[('time_ms', 'LCP')].quantile(0.95),
        'LCP length / straight': (sub[('length', 'LCP')] / sub[('straight', 'LCP')]).median(),
        'LCP length / NavFn (median)': (
            subn[('length', 'LCP')] / subn[('length', 'Navfn')]).median(),
        'NavFn length / straight': (
            subn[('length', 'Navfn')] / subn[('straight', 'Navfn')]).median(),
        'LCP mean cost': sub[('mean_cost', 'LCP')].median(),
        'NavFn mean cost': subn[('mean_cost', 'Navfn')].median(),
        'LCP turn rad': sub[('turn_rad', 'LCP')].median(),
        'NavFn turn rad': subn[('turn_rad', 'Navfn')].median(),
    })
t1 = pd.DataFrame(rows)
t1.to_csv(os.path.join(out, 'lazy_vs_fallback.csv'), index=False)
report += ['## 1. Lazy-solved vs fallback-solved requests (69 controlled maps, LCP default)', '',
           t1.to_markdown(index=False, floatfmt='.3g'), '']

# Same split per density on the 50 m density maps.
dens = ctl[['map']].drop_duplicates()
maps = w.index.get_level_values('map').to_series(index=w.index)
dcol = maps.str.extract(r'^density_(\d\d)')[0].where(maps != 'base', '10')
dcol = dcol.where(~maps.str.match(r'^density_10_s'), '10')
rows = []
for d, g in w[dcol.notna()].groupby(dcol[dcol.notna()]):
    for name, col in [('lazy', 1), ('fallback', 0)]:
        sel = (g[('valid', 'LCP')] == 1) & ((g[('valid', 'LCP_lazy')] == 1) == bool(col))
        sel_n = sel & (g[('valid', 'Navfn')] == 1)
        if sel.sum() == 0:
            continue
        rows.append({'density %': int(d) if d != '02' else 2.5, 'group': name, 'n': int(sel.sum()),
                     'LCP/NavFn length': (
                         g[sel_n][('length', 'LCP')] / g[sel_n][('length', 'Navfn')]).median(),
                     'LCP mean cost': g[sel][('mean_cost', 'LCP')].median(),
                     'LCP time ms': g[sel][('time_ms', 'LCP')].median()})
t1b = pd.DataFrame(rows)
t1b.to_csv(os.path.join(out, 'lazy_vs_fallback_by_density.csv'), index=False)
report += ['By density (50 m maps, all seeds):', '',
           t1b.to_markdown(index=False, floatfmt='.3g'), '']

# ---------------------------------------------------------------- 2. side rule
fl = load(['ablation_force_law'])
fll = load(['ablation_force_law_lazy'])
wf = wide(fl, ['valid', 'length', 'mean_cost', 'turn_rad', 'time_ms'])
wl = wide(fll, ['valid'])
both = (wf[('valid', 'r2')] == 1) & (wf[('valid', 'probe')] == 1)
r2_lazy = wl[('valid', 'r2')].reindex(wf.index) == 1
pr_lazy = wl[('valid', 'probe')].reindex(wf.index) == 1
rows = []
for name, m in [('all pairs both solve', both),
                ('both lazy', both & r2_lazy & pr_lazy),
                ('r2 lazy, probe needs fallback', both & r2_lazy & ~pr_lazy),
                ('probe lazy, r2 needs fallback', both & ~r2_lazy & pr_lazy),
                ('both need fallback', both & ~r2_lazy & ~pr_lazy)]:
    s = wf[m]
    if len(s) == 0:
        continue
    lr = s[('length', 'r2')] / s[('length', 'probe')]
    rows.append({'pairs': name, 'n': int(m.sum()),
                 'length r2/probe median': lr.median(),
                 'r2 shorter by >1 %': int((lr < 0.99).sum()),
                 'probe shorter by >1 %': int((lr > 1.01).sum()),
                 'mean cost r2': s[('mean_cost', 'r2')].median(),
                 'mean cost probe': s[('mean_cost', 'probe')].median(),
                 'turn r2': s[('turn_rad', 'r2')].median(),
                 'turn probe': s[('turn_rad', 'probe')].median(),
                 'time r2 ms': s[('time_ms', 'r2')].median(),
                 'time probe ms': s[('time_ms', 'probe')].median(),
                 'time p95 r2': s[('time_ms', 'r2')].quantile(0.95),
                 'time p95 probe': s[('time_ms', 'probe')].quantile(0.95)})
t2 = pd.DataFrame(rows)
t2.to_csv(os.path.join(out, 'side_rule.csv'), index=False)
report += ['## 2. Coulomb side (r2, default) vs nearest-free-side probe, 8 ablation maps', '',
           t2.to_markdown(index=False, floatfmt='.3g'), '']
# Tail per variant over all pairs
tails = fl[fl.valid == 1].groupby('planner').time_ms.describe(percentiles=[.5, .95, .99])
report += ['Planning time over all solved pairs (ms):', '', tails.to_markdown(floatfmt='.3g'), '']

# ---------------------------------------------------------------- 3. tails for Table I
rows = []
d50 = ctl[ctl['map'].str.match(r'density_\d\d') | (ctl['map'] == 'base')
          | ctl['map'].str.match(r'density_10_s')]
d50 = d50.copy()
d50['density'] = d50['map'].str.extract(r'density_(\d\d)')[0].fillna('10')
for (d, p), g in d50[d50.valid == 1].groupby(['density', 'planner']):
    rows.append({'density': d, 'planner': p, 'n': len(g),
                 'median': g.time_ms.median(), 'p95': g.time_ms.quantile(.95),
                 'p99': g.time_ms.quantile(.99), 'max': g.time_ms.max()})
t3 = pd.DataFrame(rows)
t3.to_csv(os.path.join(out, 'tails_density.csv'), index=False)
piv = t3.pivot(index='density', columns='planner', values=['median', 'p95', 'max'])
report += ['## 3. Planning time tails, 50 m density maps pooled over seeds '
           '(solved requests, ms)', '',
           piv.to_markdown(floatfmt='.3g'), '']
# Over all 69 maps
allt = ctl[ctl.valid == 1].groupby('planner').time_ms.describe(percentiles=[.5, .95, .99])
report += ['All 69 maps, solved requests (ms):', '', allt.to_markdown(floatfmt='.3g'), '']

open(os.path.join(out, 'report.md'), 'w').write('\n'.join(report))
print('\n'.join(report))
