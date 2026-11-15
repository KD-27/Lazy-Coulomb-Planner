# Driving test, Regulated Pure Pursuit controller (planner v2.0.0)

A simulated TurtleBot3 in headless Gazebo drives the same 15 goals in Nav2's
sandbox world (a hexagonal arena with nine pillars), once per run, with the
full Nav2 stack and each of four global planners. 10 runs per planner, strictly
one at a time on an otherwise idle machine, 15 November 2026, 02:44 to 06:03.

## Setup

- `sim/run_sim.sh <planner> 15`: native ROS 2 Jazzy, Nav2 1.3.13, parameters
  `sim/config/nav2_sim_params.yaml` (Regulated Pure Pursuit as controller, robot
  radius 0.22 m, Nav2's default behaviour tree, which replans once a second).
  Only the global planner plugin differs between runs; LCP runs with its
  defaults, including path reuse between replans.
- Goals are sampled with a fixed seed from map cells at least 0.3 m from any
  obstacle, so every run of every planner gets the same sequence.
- Each run directory `<planner>_<nn>/` holds the per-leg CSV and a trace
  (`*.trace.jsonl`: pose, every plan, velocity commands, controller warnings).
  `runs.csv` has one row per run. Summary by `sim/summarise_runs.py`.
- One run (`smac2d_03`) hung at start-up on its first attempt (navigation did
  not activate) and was repeated; the repeat is the one recorded.

## Results

| Planner | Runs | Goals reached of 15 (mean ± sd) | Runs with all 15 | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| NavFn | 10 | 14.8 ± 0.4 | 8 | 179.5 ± 11.8 | 8.0 ± 7.4 | 6.5 | 43.2 ± 1.5 | 0.25 |
| LCP | 10 | 14.1 ± 1.0 | 4 | 212.1 ± 25.0 | 39.0 ± 23.3 | 35 | 40.8 ± 1.9 | 0.22 |
| Theta* | 10 | 13.1 ± 1.1 | 0 | 216.3 ± 17.9 | 43.7 ± 18.2 | 42.5 | 41.4 ± 2.1 | 0.23 |
| Smac 2D | 10 | 10.5 ± 1.2 | 0 | 244.0 ± 20.7 | 79.5 ± 16.3 | 81 | 33.5 ± 3.0 | 0.23 |

Two-sided Mann-Whitney U tests, LCP against each other planner (p-values):

| LCP against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| NavFn | 0.062 | 0.0008 | 0.002 |
| Theta* | 0.042 | 0.41 | 0.57 |
| Smac 2D | 0.0002 | 0.001 | 0.026 |

## Findings

- **NavFn drives best with this controller.** It triggers about a fifth of
  LCP's recoveries and finishes 15 % sooner; both differences are significant.
  It reaches slightly more goals (14.8 against 14.1), which is not significant
  at 10 runs.
- **LCP and Theta\* are alike** in recoveries and time. LCP reaches one more
  goal per run on average (14.1 against 13.1, p = 0.04), and four of its ten
  runs reached every goal against none of Theta\*'s.
- **Smac 2D does clearly worst:** 10.5 goals per run and twice LCP's recoveries.
- LCP drives the shortest distance (40.8 m against 43.2 m for NavFn), consistent
  with its shorter planned paths, and passes closest to obstacles (0.22 m).
- The ordering (NavFn, then LCP and Theta\*, then Smac 2D) follows how far each
  planner keeps from obstacles: this controller stops whenever the robot is
  about 0.30 m from one, and NavFn's cost-following paths stay in the middle of
  the corridors (see `../../2026-11-14/README.md` for the trace analysis).

## Limits

- One world, and a cluttered one for a 0.22 m robot (gaps of about 0.8 m between
  pillars). In open space the planners would differ much less.
- One controller. The same planners with the MPPI controller are in
  `../driving_mppi/`; there LCP drives with almost no recoveries.
- 10 runs per planner. The spread between runs is large (LCP's recoveries range
  from 15 to 85), so only the larger differences are established.
- Simulation only.
