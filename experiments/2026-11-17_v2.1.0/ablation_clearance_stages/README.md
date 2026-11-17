# Ablation: where LCP loses clearance (planner v2.1.0)

The eight ablation maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability. Run on 17 November 2026 on `v2.1.0` compiled optimised, with the machine otherwise idle. Suite: `benchmark/suites/ablation/clearance_stages.yaml`. Full tables: `tables.md`, `summary.csv`.

Clearance is measured by `benchmark/lcp_bench.py` in metres, for the robot's centre line. It is the distance from points sampled every 5 mm along the path to the nearest edge of an obstacle cell, from a Euclidean distance transform of the map's obstacle cells (not the inflation), and it is accurate to about half a cell (2.5 cm). The robot radius is 0.22 m, so 0.22 m means the robot's edge touches an obstacle, and Nav2's inscribed zone ends at 0.224 m.

Each LCP variant runs the post-processing only up to one stage (parameters only, no code change):

| variant | lazy correction | simplification | clearance pass | Chaikin smoothing | resampling |
|---|---|---|---|---|---|
| `s0_insert` | yes | | | | |
| `s1_simplify` | yes | yes | | | |
| `s2_clearance` | yes | yes | yes | | |
| `s3_smooth` | yes | yes | yes | yes | |
| `LCP` (default) | yes | yes | yes | yes | yes |

A path from the grid fallback is always shortcut once, so fallback paths are never entirely raw in `s0_insert`.

## Findings

Over the 641 pairs that every planner solved cleanly (paired comparison):

| planner | smallest clearance, median (m) | smallest clearance, mean (m) | mean clearance, median (m) | share of path nearer than 0.5 m | length / NavFn | sharpest turn, median | time, median |
|---|---|---|---|---|---|---|---|
| NavFn | **0.425** | 0.466 | 0.976 | 0.4 % | 1.000 | 90° | 7.6 ms |
| Smac 2D | 0.378 | 0.437 | 0.892 | 1.9 % | 0.992 | 7° | 10.6 ms |
| Theta\* | 0.399 | 0.448 | 0.935 | 1.8 % | 0.983 | 45° | 9.8 ms |
| LCP, lazy correction only | 0.225 | 0.274 | 0.734 | 30.0 % | 0.980 | 77° | 0.35 ms |
| + simplification | 0.225 | 0.285 | 0.773 | 28.8 % | 0.962 | 62° | 0.35 ms |
| + clearance pass | 0.356 | 0.389 | 0.868 | 6.1 % | 0.985 | 55° | 0.50 ms |
| + smoothing | 0.356 | 0.391 | 0.844 | 8.9 % | 0.976 | 27° | 0.68 ms |
| LCP (+ resampling) | 0.356 | 0.390 | 0.855 | 9.0 % | 0.975 | 25° | 0.68 ms |

- **The prediction from the literature review was wrong.** It expected the 0.5 m waypoint margin to be undone by the simplification. In fact the lazy correction itself already leaves paths at the edge of the inscribed zone (median smallest clearance 0.225 m, 30 % of the path nearer than 0.5 m). The margin keeps the inserted *waypoint* clear, but the segments joining it to its neighbours are only required to avoid inscribed cells, so they graze obstacles. Simplification changes little (0.225 m, 28.8 %).
- **The clearance pass does almost all the work.** It raises the median smallest clearance from 0.225 m to 0.356 m and cuts the share of the path nearer than 0.5 m from 29 % to 6 %, for about 0.15 ms and 2 % more length.
- **Smoothing gives a little back.** Chaikin corner cutting leaves the smallest clearance unchanged but raises the share nearer than 0.5 m from 6 % to 9 %. In return the sharpest turn falls from 55° to 27°. Resampling changes nothing measurable.
- **After the clearance pass, LCP is close to Smac 2D and Theta\*, not to NavFn.** Its median smallest clearance is 0.356 m, against 0.378 m (Smac 2D), 0.399 m (Theta\*) and 0.425 m (NavFn). It still spends more of its length near obstacles (9 % against 2 % and 0.4 %), and its mean clearance is lower (0.86 m against 0.89 to 0.98 m). Its paths are 2.5 % shorter than NavFn's.
- **On dense maps nobody keeps clearance.** At 30 % density the median smallest clearance is 0.225 m for every LCP stage and 0.275 to 0.29 m for the grid planners. On the maze, the house and the event venue LCP keeps 0.43 to 0.55 m, against 0.47 to 0.66 m for the grid planners.

Per map, median smallest clearance (m):

| map | NavFn | Smac 2D | Theta\* | lazy only | + simplify | + clearance | + smooth | LCP |
|---|---|---|---|---|---|---|---|---|
| base | 0.608 | 0.537 | 0.514 | 0.225 | 0.244 | 0.377 | 0.387 | 0.387 |
| density 20 % | 0.387 | 0.339 | 0.366 | 0.225 | 0.225 | 0.258 | 0.258 | 0.258 |
| density 30 % | 0.275 | 0.275 | 0.291 | 0.225 | 0.225 | 0.225 | 0.225 | 0.225 |
| U shapes | 0.402 | 0.375 | 0.387 | 0.225 | 0.225 | 0.225 | 0.244 | 0.225 |
| corridor 1.2 m | 0.375 | 0.329 | 0.366 | 0.225 | 0.225 | 0.375 | 0.375 | 0.375 |
| maze | 0.655 | 0.611 | 0.627 | 0.225 | 0.230 | 0.552 | 0.552 | 0.545 |
| house | 0.475 | 0.472 | 0.462 | 0.318 | 0.337 | 0.432 | 0.432 | 0.432 |
| event venue | 0.475 | 0.475 | 0.475 | 0.225 | 0.275 | 0.425 | 0.425 | 0.425 |

## What this suggests (not tested)

The clearance pass is where clearance comes from, and it currently stops on dense maps and around U shapes (0.225 m there). Two cheap directions: run the clearance pass on the segments as well as the waypoints with a larger budget, or require each segment of the lazy correction (not only the waypoint) to keep the margin. Both are planner changes and would need a new version.

## Limits

One map per configuration and a single seed for the start/goal pairs. The comparison is over paths every planner solved, which leaves out the hardest pairs (where NavFn, Smac 2D and Theta\* fail or touch the inscribed zone). Nav2's Constrained Smoother was not tested: the benchmark has no smoother server.
