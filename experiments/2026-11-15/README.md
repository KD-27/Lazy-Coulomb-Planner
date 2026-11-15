# Experiments of 15 November 2026: planner v2.0.0

Everything here was measured on the tagged planner `v2.0.0`, unattended, between
01:00 and 08:42 on 15 November 2026, one job at a time. Unlike the development
experiments in `../2026-11-14/`, these are meant to be reproducible: each
directory records the exact suite, planner configuration and map checksums, and
has its own README with setup, findings and limits.

Machine: Intel Core i7-13620H (16 threads), 15 GB RAM, Ubuntu 24.04, ROS 2
Jazzy, Nav2 1.3.13. Costmap everywhere: robot radius 0.22 m, inflation radius
0.7 m, cost scaling 3.0.

## Planner benchmark

| Directory | What | Headline |
|---|---|---|
| `controlled_v2.0.0/` | 18 generated maps varying obstacle density (0 to 30 %), map size (10 to 200 m), obstacle shape and corridor width; LCP against NavFn, Smac 2D, Theta\*; 100 pairs per map, each timed 5 times | LCP 53× faster than the fastest grid planner on an empty 50 m map, 10× at 10 %, level at 30 %; 30× faster on a 200 m map |
| `controlled_seeds_v2.0.0/` | The density, size and U-shape maps again with seeds 2 to 5 (36 maps); `all_seeds.md` combines both suites | Speed-up 18.4 ± 0.8 at 2.5 %, 9.7 ± 0.5 at 10 %, 4.7 ± 0.8 at 20 %, 2.3 ± 0.7 at 25 %, 1.1 ± 0.3 at 30 %. LCP solved 5,365 of 5,365 routable pairs with a clean path |

## Ablations

Each compares variants of LCP that differ in one parameter, on eight maps (five
generated, three indoor), 100 pairs per map, each timed 3 times.

| Directory | Varied | Headline |
|---|---|---|
| `ablation_force_law/`, `ablation_force_law_lazy/` | Push rule: legacy probe, 1/r, 1/r², first side kept or not | Lazy search alone: 1/r² with the side kept solves 582 of 704, 1/r 539, probe 505; letting the force reverse costs 64 pairs |
| `ablation_charge_radius/`, `ablation_charge_radius_lazy/` | Range of the repulsion, 0.5 to 4 m | 4 m is 3.7× slower than 1 m and solves only 2 % more by itself |
| `ablation_retry/` | Other-side retry, its order and budget (fallback off) | No retry 77 %, default 83 %, budget 1000 89 % of routable pairs |
| `ablation_waypoint_margin/`, `ablation_waypoint_margin_lazy/` | Push past the obstacle edge, 0 to 0.5 m | Lazy search alone: 72 % with no margin, 83 % at 0.3 m, 85 % at 0.5 m; 0.5 m is also the fastest |
| `ablation_comfort_cost/` | Clearance steps off, and comfort cost 150 to 1 | Halves the cost along the path (82 to 40) for 3 % more length; saturates at 60; still short of NavFn (15) |
| `ablation_fallback_budget/` | Grid fallback off, and on with budgets 100 to 2000 | Lazy alone 83 %; with the fallback 100 % at every budget; the budget only moves the worst-case time (58 to 179 ms) |
| `ablation_smoothing/` | No simplification, no smoothing, 1 to 8 Chaikin passes | Never a colliding path; 5 passes halve the typical sharpest corner; 1 path in 10 keeps a corner over 130° |

The `_lazy` versions switch the grid fallback off, so they show what the lazy
search solves by itself; with the fallback on every variant solves everything
and only the time differs.

## Driving test

A simulated TurtleBot3 driving the same 15 goals in Nav2's sandbox world.

| Directory | What | Headline |
|---|---|---|
| `driving_rpp/` | Regulated Pure Pursuit controller, 10 runs per planner | Goals of 15: NavFn 14.8, LCP 14.1, Theta\* 13.1, Smac 2D 10.5. Recoveries per run: 8, 39, 44, 80 |
| `driving_mppi/` | MPPI controller, 5 runs per planner | LCP, NavFn and Smac 2D reach 15 of 15 in every run with about one recovery; LCP is indistinguishable from NavFn |

## What the night showed, in short

- The open-space claim holds and is quantified, with a crossover at about 30 %
  obstacle density and an advantage that grows with map size.
- With its grid fallback LCP is complete on every map tested, and returns no
  path that touches an obstacle; the grid planners do, on dense maps.
- 1/r² with the first side kept is the best push rule.
- The planner's weaknesses: paths run closer to obstacles than NavFn's, about
  one path in ten on hard maps keeps a near-reversal corner, and concave
  obstacles erase the speed advantage.
- Two default changes are supported by the data and not yet made: waypoint
  margin 0.3 to 0.5 m, and possibly a smaller attempt budget.

## Other files

`run_overnight.sh`, `run_after.sh`, `run_after2.sh` are the three chains that
ran the night's jobs in order; `overnight.log` is their timeline. One driving
run (`smac2d_03`) hung at start-up and was repeated.
