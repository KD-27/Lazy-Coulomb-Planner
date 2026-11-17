# Offline analyses for the external review (planner v2.1.0)

No new runs: computed on 17 November 2026 from the per-pair CSVs of
`../../2026-11-15_v2.1.0/` (controlled + controlled_seeds, 69 maps; ablation_force_law
and ablation_force_law_lazy, eight maps). Script: `analyse.py`; full output: `report.md`
and the CSVs next to it.

```
python3 analyse.py ../../2026-11-15_v2.1.0 .
```

## Findings

**1. Requests the fallback solves are harder, not worse served** (`lazy_vs_fallback*.csv`).
Of the 6234 routable requests on the 69 maps, the lazy correction solved 4471 and the
weighted A* fallback 1763. Median LCP/NavFn length ratio 0.98 (lazy) and 0.96 (fallback);
mean costmap cost 21 and 79, but NavFn's on the same pairs is 1.2 and 26: the fallback
pairs are the ones through clutter. Per density the gap in cost between the two groups
is 7 to 20 at equal density. Time: median 0.35 ms (lazy) vs 4.5 ms (fallback), p95 2.4 vs 13 ms.

**2. The Coulomb side rule is not better than the nearest-free-side probe** (`side_rule.csv`).
Over the 704 pairs both solve (fallback on): same median length, median mean cost 40.4
(Coulomb) vs 41.5 (probe), turn 4.1 vs 3.9 rad, median time 0.80 vs 0.53 ms, p95 5.1 vs
3.4 ms, slowest 10.4 vs 8.3 ms. On the 72 pairs that Coulomb solves lazily and the probe
sends to the fallback, the probe's paths are no longer (median ratio 1.00) and cost 42.5
vs 39.6. On the 32 pairs the other way round, the probe is better (cost 31.8 vs 36.3).
So the Coulomb rule's extra lazy solves do not buy better paths, and the probe is faster.
The Coulomb rule does beat a *fixed* side (`../../2026-11-17_v2.1.0/ablation_line_then_astar/`).

**3. Tail latency** (`tails_density.csv`). On the 50 m density maps, pooled over seeds,
LCP's p95 is 0.43 ms at 2.5 %, 2.2 at 10 %, 4.5 at 20 %, 6.7 at 30 %, 16 at 35 %, 20 at
40 %, 9.2 at 45 %; its slowest plan per density 3.7 to 28 ms. NavFn (the grid planner with
the lowest tail) has p95 24 to 42 ms and slowest 26 to 95 ms. LCP's p95 is below NavFn's
median up to 30 %. Over all 69 maps: LCP p95 6.9 ms, max 28 ms; NavFn p95 49 ms, max 518 ms;
Smac 2D 77 / 430 ms; Theta* 58 / 825 ms (maxima on the 200 m map).

## Limits

Per-pair medians of five (benchmark) or three (ablations) repeats; one machine. The
lazy/fallback split uses LCP_lazy (grid fallback off) as the oracle for "solved lazily",
which is exact because LCP runs the same lazy search before falling back.
