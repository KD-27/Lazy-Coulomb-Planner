# Ablation: is LCP more than "check the straight line, else A*"? (planner v2.1.0)

The 21 maps of the controlled suite (same generator settings and seeds as `../../2026-11-15_v2.1.0/controlled/`), 100 pairs each, every pair planned 5 times (median time used), ground-truth routability. Run on 17 November 2026 on `v2.1.0` compiled optimised, with the machine otherwise idle. Suite: `benchmark/suites/ablation/line_then_astar.yaml`. Full tables: `tables.md`, `summary.csv`, `summary_truth.csv`.

Variants (every LCP variant is the default LCP with only the listed parameters changed):

- **LCP**: the default planner.
- **line, else weighted A\*** (`line_astar_w3`): `max_iterations: 0`. The straight line is checked; the first blocked segment goes straight to LCP's own grid search (weight 3, LCP's default), followed by the same simplification, clearance pass, smoothing and resampling as LCP. This is the folk heuristic reviewers will name.
- **line, else plain A\*** (`line_astar_w1`): the same with `grid_search_weight: 1.0`.
- **fixed side** (`fixed_left`): `force_balance_threshold: 1e9`. The repulsion never decides, so every waypoint is pushed to the left of its segment first (backtracking and the fallback still work).
- NavFn and Smac 2D for reference.

The benchmark now also records clearance in metres (`clear_min_m`, smallest distance from the path to an obstacle cell edge, median over paths), from a distance transform of the map's obstacle cells.

## Findings

Median planning time in ms (95th percentile in brackets):

| map | NavFn | Smac 2D | LCP | line, else weighted A\* | line, else plain A\* | fixed side |
|---|---|---|---|---|---|---|
| empty | 11.0 | 9.2 | 0.07 | 0.07 | 0.06 | 0.07 |
| 2.5 % | 15.0 | 13.8 | **0.16** (0.47) | 0.42 (0.74) | 3.6 | 0.13 |
| 5 % | 15.5 | 13.6 | **0.25** (0.59) | 0.48 (0.90) | 4.5 | 0.27 |
| 10 % (base) | 14.1 | 15.8 | **0.43** (1.38) | 0.67 (1.20) | 5.3 | 0.71 |
| 15 % | 15.6 | 18.6 | **0.63** (3.26) | 0.80 (1.29) | 4.7 | 1.05 |
| 20 % | 13.6 | 15.1 | 1.17 (4.31) | **0.90** (1.43) | 4.7 | 2.45 |
| 25 % | 12.6 | 14.2 | 2.91 (5.70) | **1.11** (3.01) | 4.6 | 3.33 |
| 30 % | 13.6 | 17.0 | 3.83 (6.18) | **1.47** (3.41) | 5.4 | 3.88 |
| 35 % | 16.2 | 23.4 | 4.16 | **1.89** | 5.9 | 4.51 |
| 40 % | 18.5 | 18.6 | 5.56 | **2.98** | 5.8 | 5.72 |
| 45 % | 14.7 | 11.0 | 5.27 | **2.82** | 3.9 | 5.35 |
| 10 m map | 0.76 | 1.42 | **0.10** | 0.13 | 0.30 | 0.08 |
| 25 m map | 4.06 | 4.95 | **0.21** | 0.29 | 1.69 | 0.29 |
| 100 m map | 52.2 | 54.5 | **0.73** (2.73) | 2.01 (3.06) | 20.2 | 1.40 |
| 200 m map | 304 | 157 | **3.29** (10.2) | 4.64 (9.03) | 102 | 5.68 |
| squares | 14.5 | 13.4 | **0.40** | 0.72 | 5.1 | 0.66 |
| U shapes | 14.7 | 18.8 | 2.83 | **1.14** | 6.5 | 3.54 |
| corridor 0.8 m | 4.75 | 10.3 | 1.63 | **0.83** | 2.7 | 1.65 |
| corridor 1.2 m | 8.03 | 14.4 | **0.69** | 0.85 | 3.6 | 1.24 |
| corridor 2.0 m | 8.00 | 13.8 | **0.57** | 0.86 | 5.6 | 1.25 |
| corridor 3.0 m | 8.94 | 11.6 | **0.35** | 0.59 | 4.9 | 0.87 |

- **The lazy repair pays only where obstacles are sparse.** Against "line, else weighted A\*", LCP is faster on open maps (1.3x at 15 %, 1.6x at 10 %, 2.6x at 2.5 %), on large maps (2.7x at 100 m, 1.4x at 200 m), with square obstacles and in corridors of 1.2 m and wider. From 20 % density, with U shapes and in 0.8 m corridors, the folk heuristic is **faster** (1.3x at 20 %, 2.6x at 30 %), and its 95th percentile is lower almost everywhere. The crossover against this baseline is between 15 % and 20 % density, well below the 30 % crossover against Nav2's planners.
- **Most of the speed-up over Nav2 comes from the line check plus LCP's weighted A\*, not from the repair.** "Line, else weighted A\*" is itself 4x to 65x faster than NavFn and Smac 2D on every map. Against "line, else plain A\*" (weight 1) LCP is 7x to 31x faster on maps up to 15 % density and from 25 m up (3x on the 10 m map), but only 1.4x to 4x at 20 % to 30 %, and on par at 40 % to 45 %. Weighting the grid search therefore buys more than the repair does.
- **The Coulomb side choice helps.** Pushing always to the left first is slower than LCP on most maps (1.7x at 10 %, 2.1x at 20 %, 1.9x at 100 m, 1.7x at 200 m) and gives longer paths in corridors (1.03 to 1.08 of NavFn's length against 0.97 to 1.01 for LCP). It is slightly faster only on the emptiest and smallest maps, where there is almost nothing to choose.
- **Path length is not the difference.** The folk heuristic's paths are as short as LCP's (shortcut grid paths): median length relative to NavFn 0.93 to 1.01 for both.
- **Every LCP variant solves every routable pair** (it is the same fallback). NavFn and Smac 2D leave some routable pairs unsolved on the dense maps and the 0.8 m corridor, and Smac 2D solves only 58 of 100 pairs on the 200 m map.
- **Clearance** (median of each path's smallest clearance): NavFn keeps the most (0.60 m on the base map, against 0.38 m for LCP and 0.29 m for the folk heuristic). On maps of 30 % density and above every planner is down to the 0.22 m robot radius. See `../ablation_clearance_stages/` for where LCP loses it.

## What this means for the paper

LCP should not be presented as faster than A\* in general. Its contribution splits into two parts: (i) a line check and a weighted A\* on the costmap, with shortcutting and a clearance pass, which already beats Nav2's planners by one to two orders of magnitude; and (ii) the lazy repair with the Coulomb side choice, which adds a further 1.3x to 2.7x on sparse and large maps and costs up to 2.6x on dense ones. A density-dependent switch (try the repair only while few corrections have been needed) is suggested by these numbers but not tested.

## Limits

One map per configuration and a single seed for the start/goal pairs. "Line, else A\*" here uses LCP's own grid search and post-processing; a different A\* implementation would give different absolute times.
