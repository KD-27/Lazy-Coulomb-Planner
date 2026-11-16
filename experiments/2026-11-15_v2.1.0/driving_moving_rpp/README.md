# Driving test with moving obstacles, Regulated Pure Pursuit, planner v2.1.0

The TurtleBot3 driving test with obstacles that move, and with Nav2 replanning at
its default 1 Hz or at 5 Hz, to see whether a faster planner pays off when plans
are frequent. Two 20 m worlds (open; 10 % static cylinders), 4 planners, 2
replanning rates, 3 runs each, 10 goals per run: 48 runs, one at a time,
16 November 2026, 09:46 to 14:28, plus one repeated run at 14:34.

## Setup

- Worlds from `sim/make_worlds.py sim/worlds/moving_set.yaml` (in `worlds/`), then
  `sim/add_moving_obstacles.py --count 6 --speed 0.3`: six upright cylinders
  (radius 0.25 m) move back and forth at 0.3 m/s on straight lanes 3 to 6 m long,
  driven by `sim/move_obstacles.py`. They are not in the map; Nav2 sees them only
  through the lidar (local and global costmap obstacle layers).
- Replanning rate: the `RateController` of Nav2's
  `navigate_to_pose_w_replanning_and_recovery` behaviour tree, set to 1 Hz
  (default) or 5 Hz by `LCP_SIM_REPLAN_HZ`.
- Planner CPU: CPU seconds used by the `planner_server` process over the run
  (`planner_cpu_s.txt`). This includes the global costmap updates, which are the
  same work for every planner, so differences between planners are diluted.
- **Localisation: AMCL**, not the wheel odometry used in `../driving_worlds_rpp/`.
  This was an oversight (`run_moving.sh` was written before the generated-world
  runs switched to odometry and was not updated). The moving cylinders are not in
  AMCL's map, so some recoveries and missed goals may come from localisation
  error rather than planning. All four planners ran the same way, so the
  comparison between them is like for like, but the absolute numbers are not
  comparable with `../driving_worlds_rpp/`.
- Leg timeout 150 s of simulated time; run timeout 2400 s.
- Start-up failures, repeated automatically: `open/1hz/smac2d_01`,
  `open/1hz/lcp_03`, `open/5hz/navfn_03` (localisation did not start),
  `density_10/1hz/smac2d_03`.
- **One invalid run, replaced:** in the first `open/1hz/theta_03`, Nav2's
  `collision_monitor` crashed (segfault) during leg 2; the lifecycle manager shut
  navigation down, and `sim/drive.py` then recorded legs 3 to 9 as reached with
  0 s and 0 m. It is kept in `invalid/open_1hz_theta_03_navstack_crash/` and was
  rerun (10/10 goals). `sim/drive.py` now aborts a run, without writing results,
  when a goal more than 0.3 m away is reported reached in under 0.5 s of sim
  time, so the batch script retries it. No other run in this directory, or in
  the other driving directories of this version, has such legs.

## Results (mean ± sd over 3 runs; `runs_all.csv` has every run)

| World, replanning | Planner | Goals of 10 | Recoveries | Sim time (s) | Planner CPU (s) |
|---|---|---|---|---|---|
| open, 1 Hz | LCP | 10.0 ± 0 | 6.3 ± 3.8 | 279 ± 6 | 15.3 ± 0.2 |
| | NavFn | 10.0 ± 0 | 1.3 ± 2.3 | 277 ± 5 | 14.5 ± 2.0 |
| | Smac 2D | 10.0 ± 0 | 5.3 ± 3.2 | 280 ± 2 | 15.5 ± 0.9 |
| | Theta\* | 10.0 ± 0 | 6.0 ± 1.7 | 275 ± 6 | 15.6 ± 1.1 |
| open, 5 Hz | LCP | 10.0 ± 0 | 5.7 ± 2.5 | 275 ± 4 | 14.9 ± 1.1 |
| | NavFn | 10.0 ± 0 | 4.0 ± 0 | 285 ± 11 | 17.4 ± 1.9 |
| | Smac 2D | 10.0 ± 0 | 3.3 ± 0.6 | 278 ± 1 | 19.1 ± 2.1 |
| | Theta\* | 10.0 ± 0 | 11.3 ± 5.0 | 283 ± 6 | 16.5 ± 0.2 |
| 10 % density, 1 Hz | LCP | 9.7 ± 0.6 | 15.7 ± 14.2 | 355 ± 55 | 17.5 ± 2.2 |
| | NavFn | 10.0 ± 0 | 1.7 ± 2.1 | 347 ± 1 | 19.6 ± 0.5 |
| | Smac 2D | 9.0 ± 1.7 | 7.0 ± 3.6 | 322 ± 51 | 17.6 ± 4.6 |
| | Theta\* | 10.0 ± 0 | 4.3 ± 4.0 | 353 ± 9 | 19.7 ± 2.4 |
| 10 % density, 5 Hz | LCP | 10.0 ± 0 | 9.7 ± 6.7 | 346 ± 13 | 19.7 ± 3.2 |
| | NavFn | 10.0 ± 0 | 1.3 ± 1.5 | 346 ± 2 | 20.9 ± 0.8 |
| | Smac 2D | 9.3 ± 1.2 | 8.0 ± 6.2 | 338 ± 10 | 20.4 ± 1.0 |
| | Theta\* | 9.0 ± 0 | 5.7 ± 4.5 | 318 ± 20 | 20.0 ± 3.4 |

Mann-Whitney U, two-sided, LCP against each other planner: no p-value is below
0.10, which is the smallest possible with 3 runs against 3. The per-condition
tables with tests are in `<world>/<rate>hz/summary.md`.

## Findings

- Moving obstacles at 0.3 m/s do not separate the planners on success: all four
  reach 9 to 10 of 10 goals in every condition.
- Planner CPU while replanning at 5 Hz in the open world: LCP 14.9 s, NavFn 17.4,
  Theta\* 16.5, Smac 2D 19.1. Going from 1 Hz to 5 Hz, LCP's CPU stays the same
  (15.3 to 14.9 s) while the grid planners' rises by 0.9 to 3.6 s. In the 10 %
  world the pattern does not hold clearly (LCP +2.2 s, NavFn +1.3, Smac 2D +2.8,
  Theta\* +0.3) and the spread is larger than the differences. Because the
  measure includes the shared costmap work, it understates the planner's own
  share; the planner-only benchmark (`../controlled/`) is the clean measure of
  planning time.
- Faster replanning does not make any planner arrive sooner: sim times at 5 Hz
  are within the spread of those at 1 Hz.
- LCP triggers more recoveries than NavFn in every condition (e.g. 15.7 against
  1.7 at 10 %, 1 Hz, where one LCP run alone had 32), consistent with the
  clearance finding in the other Regulated Pure Pursuit tests; not significant
  with 3 runs.

## Limits

- 3 runs per condition; nothing here is statistically significant.
- AMCL localisation with unmapped moving obstacles (see Setup).
- One controller (Regulated Pure Pursuit), one obstacle speed, six movers,
  20 m worlds, simulation only.
- Planner CPU includes the global costmap updates.
