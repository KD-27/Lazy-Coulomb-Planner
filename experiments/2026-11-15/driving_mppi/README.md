# Driving test, MPPI controller (planner v2.0.0)

The same test as `../driving_rpp/` (TurtleBot3, Nav2's sandbox world, the same
15 goals) with one change: the controller is MPPI instead of Regulated Pure
Pursuit. 5 runs per planner, one at a time, 15 November 2026, 06:03 to 07:26.

## Setup

- Run inside the Docker image `lcp-jazzy:latest`, because `nav2_mppi_controller`
  is not installed natively on this machine. The workspace built natively is
  mounted at the same path; ROS 2 Jazzy and Nav2 1.3.13 as on the host.
- `LCP_SIM_CONTROLLER=mppi sim/run_sim.sh <planner> 15`, parameters
  `sim/config/nav2_sim_params_mppi.yaml`: identical to the Regulated Pure Pursuit
  set except for the controller block, which is the MPPI configuration of
  `nav2_bringup`.
- No run needed a second attempt.

## Results

| Planner | Runs | Goals reached of 15 (mean ± sd) | Runs with all 15 | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| LCP | 5 | 15.0 ± 0.0 | 5 | 219.9 ± 20.5 | 1.0 ± 2.2 | 0 | 48.0 ± 0.9 | 0.25 |
| NavFn | 5 | 15.0 ± 0.0 | 5 | 215.7 ± 6.6 | 1.0 ± 0.7 | 1 | 49.1 ± 0.5 | 0.29 |
| Smac 2D | 5 | 15.0 ± 0.0 | 5 | 193.2 ± 5.7 | 0.2 ± 0.4 | 0 | 47.1 ± 0.3 | 0.22 |
| Theta* | 5 | 14.6 ± 0.9 | 4 | 233.0 ± 95.9 | 6.8 ± 14.1 | 0 | 47.7 ± 1.2 | 0.22 |

Two-sided Mann-Whitney U tests, LCP against each other planner (p-values):
goals reached 1.0, 1.0, 0.42; recoveries 0.26, 1.0, 0.61; simulated time 1.0,
0.008, 0.15 (against NavFn, Smac 2D, Theta\*).

## Findings

- **With MPPI every planner drives the course.** LCP, NavFn and Smac 2D reach
  all 15 goals in all 5 runs with about one recovery per run or fewer. Theta\*
  has one bad run (13 goals, 32 recoveries) and four clean ones.
- **LCP is indistinguishable from NavFn** here: same goals, same recoveries, same
  time (220 s against 216 s).
- **Smac 2D is the fastest** (193 s, significantly faster than LCP), the opposite
  of its last place with Regulated Pure Pursuit.
- Compared with `../driving_rpp/`, LCP's recoveries drop from 39 per run to 1.
  The stops seen there are a property of that controller meeting paths that pass
  near obstacles, not of the planner: MPPI steers around the same situations.

## Limits

- 5 runs per planner; enough to show that everything works, not to rank the
  planners finely.
- One world, simulation only.
- Run in Docker and not natively, unlike the Regulated Pure Pursuit batch. The
  software versions are the same, but timing conditions may differ slightly.
