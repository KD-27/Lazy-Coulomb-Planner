# Development of planner v2.1 (15 November 2026)

Step-by-step measurements made while changing the planner after `v2.0.0`. Each
step ran the development suite (`benchmark/suites/dev*.yaml`): the eight
ablation maps, 100 pairs each, one timing per pair. These are development
numbers, taken on uncommitted code; the reproducible results for `v2.1` will
come from rerunning the full suites on the tag.

**Build note.** Up to and including step 3 the planner was compiled without
optimisation: the package set no build type, so CMake added no `-O` flag, while
the Nav2 planners are optimised release builds. Every LCP timing before step 4,
including all of `../controlled_*`, `../ablation_*` and `../../2026-11-14`, is
therefore pessimistic by a factor of about 2.5. Success, clearance and
path-shape figures are not affected. The package now defaults to a Release
build.

| Directory | Change measured | Result |
|---|---|---|
| `step1_shortcut_margin_resample/` | Simplification joins each waypoint to the farthest one reachable in a straight line; waypoint margin 0.3 to 0.5 m; output resampled evenly | Paths with a corner over 90°: 104 of 704 (15 %) to 21 (3 %). Lazy search alone solves 601 against 582. Poses per path halved. The sharp corners came from tangles of waypoints the lazy search left around single obstacles, which neighbour-only simplification could not remove |
| `step2_astar_fallback/` | Fallback route from a weighted A\* instead of the breadth-first flood, weights 1, 2, 4 | First version re-opened cells: slowest plan up to 490 ms on the maze (38 ms before). No gain in typical time |
| `step2b_astar_settled/` | Same, each cell settled once | Pathological cases gone; typical time on hard maps unchanged. Timing the stages showed why: of 34 ms for a fallback pair on the 30 % map, 23 ms was the lazy search using up its 300 attempts and 8 ms the grid search |
| `step3_budget/` | Attempt budget 300, 150, 100, 50, 25 with the A\* fallback | A smaller budget is faster on hard maps; the grid search becomes the floor |
| `step4_release_build_budget/` | The same sweep with the planner compiled optimised (`-O3`) | Mean of the per-map median time 1.2 to 1.6 ms (it was about 4 ms unoptimised); base map 0.45 ms against NavFn's 14.3 ms |
| `step5_budget_and_weight/` | Budget 35, 50, 75 with A\* weights 2, 3, 4 | All close. Chosen: budget 50, weight 3 (mean median 1.20 ms, slowest plan 11.7 ms). The lazy search alone then solves 489 of 704 |
| `baseline_v2.0.0_release_build/` | The tag `v2.0.0` built optimised, for a fair comparison | Mean median 2.05 ms, slowest plan 19 ms, 104 paths with a corner over 90° |
| `step6_candidate_v2.1/` | The candidate with its new defaults | Mean median 1.20 ms, slowest plan 9 ms, 13 paths with a corner over 90°, 704 of 704 solved |
| `driving_rpp_v2.1_candidate/` | Five driving runs, Regulated Pure Pursuit | See below |

## v2.0.0 against the v2.1 candidate, both optimised

| | v2.0.0 | v2.1 candidate |
|---|---|---|
| Routable pairs solved | 704 of 704 | 704 of 704 |
| Solved by the lazy search alone | 582 | 489 |
| Paths with a corner over 90° | 104 (15 %) | 13 (2 %) |
| Mean of the per-map median time | 2.05 ms | 1.20 ms |
| Slowest plan | 19 ms | 9 ms |
| Median on the 30 % density map | 7.7 ms | 3.4 ms |
| Median on the U-shaped obstacles | 4.9 ms | 2.4 ms |
| Mean path length | 24.6 m | 23.6 m |
| Mean cost along the path | 43.0 | 42.1 |

The lazy search does less of the work in v2.1 (69 % of these hard pairs against
83 %), because the budget is cut from 300 to 50 attempts and the A\* fallback
takes over sooner. Overall that is faster.

A first run of the `v2.0.0` baseline was built from the wrong commit (an older
one left in the comparison workspace) and was discarded.
