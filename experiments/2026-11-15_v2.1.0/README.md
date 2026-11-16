# Experiments on planner v2.1.0 (15 and 16 November 2026)

Everything here was measured on the tagged planner `v2.1.0`, compiled optimised
(Release), one job at a time on an otherwise idle machine (Intel Core i7-13620H,
15 GB RAM, Ubuntu 24.04, ROS 2 Jazzy, Nav2 1.3.13). Each directory has its own
README with setup, findings and limits. `run.log` is the timeline;
`run_all.sh`, `run_after.sh` and `run_night2.sh` (with `run_worlds.sh` and
`run_moving.sh`) are the scripts that ran it.

Unlike `../2026-11-15/` (v2.0.0), whose timings came from an unoptimised build,
these timings are representative.

## Planner benchmark

| Directory | What | Headline |
|---|---|---|
| `controlled/` | 21 generated maps: density 0 to 45 %, size 10 to 200 m, square and U-shaped obstacles, corridors 0.8 to 3 m | LCP 30× faster than the fastest grid planner on the 50 m base map, 45× on 200 m; slowest plan 13 ms |
| `controlled_seeds/` | The density, size and U-shape maps with four more seeds; `all_seeds.md` combines all five | 66× faster at 2.5 % density, 31× at 10 %, 9× at 20 %, 4× at 30 %, 2.6× at 40 %; no crossover up to 45 %. 6,234 of 6,234 routable pairs solved cleanly, 666 of 666 unroutable refused |

## Ablations (eight maps, 704 routable pairs each)

| Directory | Headline |
|---|---|
| `ablation_force_law/`, `_lazy/` | Coulomb side choice: 489 pairs solved by the lazy search alone against 449 for the nearest-side rule; with the fallback the simple rule is fastest. Letting the force reverse the waypoint is worst |
| `ablation_charge_radius/`, `_lazy/` | Larger radius: a few more lazy successes at many times the cost |
| `ablation_retry/` | With a budget of 50 the retry order hardly matters; the budget decides (25: 405, 300: 601 lazy successes) |
| `ablation_waypoint_margin/`, `_lazy/` | Strongest lever: 305 lazy successes without a margin, 489 at 0.5 m |
| `ablation_comfort_cost/` | Clearance steps halve the cost along the path for 3 % more length; saturates at 60 |
| `ablation_fallback_budget/` | Budgets 10 to 50 alike; 300 doubles the slowest plan |
| `ablation_grid_search_weight/` | Unweighted A* fallback triples the 95th percentile; any weight from 2 is fine |
| `ablation_smoothing/` | Farthest-visible simplification removes most sharp corners; 3 Chaikin passes suffice |

## Driving (TurtleBot3, Gazebo)

| Directory | What | Headline |
|---|---|---|
| `driving_rpp/` | Sandbox world, Regulated Pure Pursuit, 10 runs per planner | Goals of 15: NavFn 14.7, LCP 14.3, Theta* 12.5, Smac 2D 11.4; recoveries per run 10, 35, 53, 67 |
| `driving_mppi/` | Sandbox world, MPPI, 5 runs per planner | LCP 15/15 in every run with 0 recoveries and the shortest time; NavFn 15/15 with 4 recoveries |
| `driving_worlds_rpp/` | Six generated worlds, Regulated Pure Pursuit, 3 runs per planner, odometry localisation | All planners perfect in open space, at 10 % density and with U shapes; at 20 % and in corridors LCP and Theta* need many recoveries, NavFn few |
| `driving_worlds_rpp_amcl_aborted/` | First attempt at the above with AMCL, stopped after 9 runs | AMCL mislocalised the robot among identical cylinders |
| `driving_moving_rpp/` | Moving obstacles, 1 and 5 Hz replanning | Not run: on hold at the author's request (`HOLD_MOVING`). Worlds are built; `./run_moving.sh` starts it after removing the hold file |
