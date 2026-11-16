# Driving test, MPPI controller, planner v2.1.0

The same test as `../driving_rpp/` with the MPPI controller instead of Regulated
Pure Pursuit, run inside the Docker image `lcp-jazzy:latest` (MPPI is not
installed natively on this machine; same ROS 2 and Nav2 versions). 5 runs per
planner, one at a time, 15 November 2026, 21:22 to 22:45.

| Planner | Goals reached of 15 | Runs with all 15 | Sim time (s) | Recoveries per run | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|
| LCP | 15.0 ± 0.0 | 5 | 185 ± 5 | 0.0 ± 0.0 | 47.1 | 0.24 |
| NavFn | 15.0 ± 0.0 | 5 | 248 ± 38 | 4.2 ± 4.0 | 49.5 | 0.29 |
| Smac 2D | 14.8 ± 0.4 | 4 | 213 ± 49 | 2.6 ± 5.8 | 46.7 | 0.22 |
| Theta\* | 14.8 ± 0.4 | 4 | 219 ± 27 | 8.4 ± 11.6 | 48.0 | 0.22 |

Mann-Whitney U, LCP against NavFn: recoveries p = 0.025, time p = 0.008;
against Theta\*: recoveries p = 0.007, time p = 0.008.

## Findings

- With MPPI, LCP is the best of the four: all goals in every run, no recovery
  behaviour in any run, and the shortest time (185 s against 248 s for NavFn).
  The differences in recoveries and time against NavFn and Theta\* are
  significant even with 5 runs.
- Together with `../driving_rpp/` this shows that LCP's paths are well suited to
  a model-predictive follower, which plans its own motion around nearby
  obstacles, and less so to a follower that stops whenever the path runs close
  to one.

## Note

`smac2d_04` ended with a shell error ("unexpected EOF") because `sim/run_sim.sh`
was edited while the run was using it. Its data is complete (15/15 goals,
0 recoveries) and is included.

## Limits

5 runs per planner, one world, simulation only, run in Docker.
