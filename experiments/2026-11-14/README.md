# Experiments of 14 November 2026

Everything measured during the day the planner was ported to Jazzy and reworked.
These are **development experiments**: each compared options on whatever the
code was at that moment, to decide what to build next. Most were run on
uncommitted intermediate code and cannot be reproduced exactly. They explain
why the defaults are what they are; they are not paper-grade results. The
reproducible results for the committed code are in `benchmark/results/` and
`sim/results/`.

Machine: Intel Core i7-13620H (16 threads), 15 GB RAM, Ubuntu 24.04, ROS 2
Jazzy, Nav2 1.3.13.

## Planner benchmarks (`planner/`)

Produced by `benchmark/lcp_bench.py`: 100 random start/goal pairs per map with a
fixed seed, on a costmap with 0.22 m robot radius and 0.7 m inflation. One CSV
per map; every row is one planner on one pair. NavFn is the reference in every
file. The column meanings are in `benchmark/README.md` (early files have fewer
columns: sharpest corner and cost columns were added during the day). The
planner names inside each CSV are defined by the matching file in `configs/`.

| Folder | What was varied | Config | Result |
|---|---|---|---|
| `01_force_model` | Push rule: legacy probe, 1/r, 1/r², each Coulomb law with and without keeping the first side | `force_models.yaml` | Pure Coulomb oscillates between obstacles and is worse than the probe (maze 33 and 32 of 100 against 42). Keeping the first side fixes it; 1/r² then wins or ties everywhere (maze 61 against 43 for 1/r). |
| `02_exponent_x_radius` | Exponent 1 or 2 × charge radius 0.5, 1, 2, 4 m, side kept | `radius_sweep.yaml` | Radius matters as much as the exponent. 1/r² is ahead at 0.5–2 m; 1/r is ahead only at 4 m, where planning is 5–10× slower. Default: 1/r², 1 m. |
| `03_baseline_before_retry` | Nothing: the state before the retry work | `benchmark/config/planners.yaml` | Maze 61/100, event venue 48/54, house 51/57; sharpest corner on a typical path 13–49°. |
| `04_retry_first_attempt` | Other-side retry on/off | `planners.yaml` with an extra `LCP_noretry` entry | Retry helped little (maze 50 → 61). The "no progress" check was too strict in this version, so the numbers understate the no-retry case. Superseded by `05`. |
| `05_retry_order_and_budget` | Retry order (most recent / shortest partial path) × budget 500 or 2000 | `retry.yaml` | Shortest-first is clearly better: maze 72 against 61 at budget 500, 88 at 2000. |
| `06_waypoint_margin` | Extra push past the obstacle edge: 0, 0.15, 0.3, 0.5 m | `margin.yaml` | 0.3 m improves success (maze 88 → 98), speed and smoothness together. Default 0.3 m. |
| `07_comfort_first_attempt` | Comfort level off, 150, 100, 50, 1 | `comfort.yaml` | Clearance improved, but the extra waypoints produced hairpins (sharpest corner 26–46°). Led to the offset limit and smoothing slack. Superseded. |
| `08_smoothing_slack` | Clearance off; slack 0; slack 40 (after the hairpin limit) | `slack.yaml` | With the limit and slack 40 corners return to 4–10° and the worst cost on a typical path falls from about 230 to 130–170. The first run of this sweep (six slack values, before the limit) was overwritten. |
| `09_fallback_budget` | Grid fallback off, and on with budget 2000, 1000, 300, 100 | `fallback.yaml` | With the fallback every routable pair is solved; failures are reported in about 20 ms instead of up to 180 ms. Budget 300 is the best balance. This is the second run, after the corner-point traversal fix; the first run was overwritten. |
| `10_comfort_before_sliding` | Comfort 100, 60, 30, 1 × slack 40 or 20 | `comfort2.yaml` | Almost no effect: the closest points were the corner waypoints, which the clearance pass could not move. |
| `11_comfort_after_sliding` | Same, after existing waypoints were allowed to slide along the Coulomb force | `comfort2.yaml` | Clear effect. Comfort 30, slack 40: worst cost on a typical path 173 → 129 (event venue), 177 → 92 (maze), 153 → 128 (house). Default comfort 30. |

Not kept as CSV, recorded here for completeness:

- **First Jazzy check, original Humble algorithm.** Phase 4 start/goal pairs on
  `event_venue`: LCP returned a path for 10/10, of which 2 were collision free;
  5 of the pairs had no route at all. NavFn solved the 5 routable pairs.
- **Nav2's upstream benchmark, original algorithm, 100 m map at 10 %.** LCP
  median 3.2 ms against 39–54 ms for the grid planners, but 76 of 100 LCP paths
  touched obstacle cells.
- **Humble Phase 4 tuning sweep** (on the `evaluation` branch): all 270 runs
  returned the untouched straight line, so that dataset is void.

## Driving tests (`driving/`)

Produced by `sim/drive.py`: a TurtleBot3 in headless Gazebo driving the same 15
goals in Nav2's sandbox world. One CSV per run, one row per leg. Results vary a
lot from run to run, and a busy machine makes them worse, so single runs prove
little.

| Folder | Setup | Result |
|---|---|---|
| `docker_mppi_clearance_run1`, `_run2` | Docker, MPPI controller, LCP with the first clearance version | 15/15 goals; 268 s and 263 s; 5 and 4 recoveries. |
| `docker_mppi_fallback_run2` | Docker, MPPI, LCP with the grid fallback | 15/15; 288 s; 6 recoveries. The first run of this pair (318 s, 10 recoveries) was overwritten. |
| `docker_mppi_pairs` | Docker, MPPI, each simulation isolated; `old` = clearance commit, `new` = fallback code, run side by side | old 234, 237, 228 s with 0 recoveries; new 220, 221, 236 s with 0, 1, 1. No difference beyond noise. |
| `native_rpp_first_3goals` | First native run, Regulated Pure Pursuit controller, 3 goals | 3/3 goals. Smoke test only. |
| `native_rpp_baseline` | Native, Regulated Pure Pursuit, LCP and NavFn side by side | LCP 15/15, 187 s, 16 recoveries; NavFn 15/15, 189 s, 9 recoveries. |
| `native_rpp_parallel_pairs` | Native; `old` = commit 671e2c3, `new` = waypoint sliding and comfort 30 (no goal tolerance), side by side | old 12, 15, 15 of 15 goals with 45, 25, 37 recoveries; new 14, 13, 15 with 38, 26, 10. **Unreliable**: lint and tests ran on the machine during the first pair, and two simulations share the CPU. Four requests failed with "goal is inside an obstacle", which led to the goal tolerance. |
| `native_rpp_sequential` | Native, one run at a time; `new` also has the goal tolerance | new 15 and 14 of 15 with 22 and 45 recoveries; old 14 and 14 with 45 and 42. **Incomplete**: the third run hung because Gazebo started before its world file was written (since fixed), and the batch was stopped. |

| `native_rpp_sequential_rerun` | Native, one run at a time on an idle machine, after the Gazebo start-up race was fixed. `old` = commit 671e2c3; `new` = waypoint sliding, comfort 30 and goal tolerance; plus NavFn | new: 15, 15, 14 of 15 goals, 234, 203, 201 s, 40, 20, 33 recoveries. old: 13, 14, 15 goals, 209, 243, 189 s, 36, 51, 22 recoveries. NavFn: 15 and 15 goals, 172 and 184 s, 1 and 9 recoveries. New and old do not differ beyond noise in recoveries; new reaches more goals (44 against 42 of 45) because the goal tolerance removes the "goal is inside an obstacle" failures. LCP triggers several times more recoveries than NavFn with this controller, always on the same legs (2, 6, 7, 9 to 11, 13). |

| `native_rpp_rounded_corners` | Native, sequential. LCP with the smoothing rule that lets a corner be rounded as close as the adjoining stretches already are, and deeper cuts | 14 and 13 of 15 goals, 208 and 243 s, 33 and 53 recoveries. No improvement: tight corners were not the cause of the stops. A third run did not start (the empty-domain check fired too early, since fixed). Two earlier runs of this batch were discarded: stray transform publishers left by the path analysis were on the same ROS domain. |
| `native_rpp_path_reuse` | Native, sequential. As above plus keeping the previous path between replans | 13 and 14 of 15 goals, 258 and 202 s, 76 and 27 recoveries. No improvement either. A third run hung at start-up (map server stuck configuring) and was discarded. |
| `native_rpp_traced` | Native, one run each of LCP, NavFn, Smac 2D and Theta*, with the new trace (`*.trace.jsonl`: robot pose, every published plan, velocity commands, controller warnings) | LCP 13/15 goals, 53 recoveries, 432 "collision ahead" warnings. NavFn 15/15, 0 recoveries, 0 warnings. Smac 2D 5/15, 21 recoveries, 153 warnings. Theta* 12/15, 47 recoveries, 352 warnings. |

Earlier Docker runs with the MPPI controller are only in the git history of
`sim/results/`: LCP before the start-cell fix reached 10/15 goals; after it,
15/15 in 316 s with 11 recoveries; NavFn 15/15 in 254 s with 5 recoveries.

Conclusion: with the Regulated Pure Pursuit controller in this pillar world the
robot stops for "collision ahead" on the paths of every planner except NavFn.
The trace shows why: the stops happen when the robot itself is about 0.30 m from
an obstacle (its radius is 0.22 m), and the robot spends 18 % of the run that
close on LCP paths against 4 % on NavFn paths, whose cost-following route keeps
to the middle of the corridors (median clearance 0.45 m against 0.35 m). Smac 2D
and Theta*, which also take short routes, do no better than LCP. Tight corners
and path jumps between replans were both tested as causes and ruled out as the
main one. With the MPPI controller (Docker runs) LCP drove with 0 to 1
recoveries. So this test is harsh on every planner that does not centre itself
in corridors; it does not single out LCP.

## Path analysis (`analysis/`)

Planner-only measurements on the sandbox map, over the 15 legs of the driving
test. `leg_paths.py` plans each leg and reports length, clearance and the
tightest turn; `replan_stability.py` replans from points along a path, nudged
5 cm sideways, and reports how far the new path moves.

| File | What it shows |
|---|---|
| `leg_paths.csv`, `.png` | Before corner rounding: LCP paths are boxy, tightest turn 0.09 to 0.21 m on most legs against 0.26 to 0.36 m for NavFn. |
| `leg_paths_rounded.*`, `leg_paths_deeper_cuts.*` | After the smoothing rule change, then with deeper cuts: tightest turn 0.20 to 0.56 m on all legs but one; total length 45.6 m against 45.9 m for NavFn. |
| `leg_paths_with_reuse.*` | Final code, three planners. Length closer than 0.33 m to an obstacle: LCP 2.1 m, NavFn 0.9 m, Smac 2D 9.9 m. Median clearance: 0.40, 0.45, 0.40 m. |
| `replan_stability_before_reuse.csv` | LCP: 6 of 133 replans move the path by more than 0.3 m (legs 6, 7, 9, 12, 13). NavFn: 0 of 141. |
| `replan_stability_with_reuse.csv` | With path reuse LCP has 0 jumps, and its paths move less between replans than NavFn's (90th percentile 0.056 m against 0.076 m). |
