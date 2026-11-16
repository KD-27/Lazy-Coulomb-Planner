# Driving test with moving obstacles, wheel-odometry localisation, planner v2.1.0

A repeat of `../driving_moving_rpp/` with one change: the robot's pose comes from
wheel odometry (no AMCL), as in `../driving_worlds_rpp/`. The first batch used
AMCL by mistake, and AMCL matches the lidar against a map that does not contain
the moving cylinders, so some of its recoveries and missed goals could have been
localisation errors rather than planning. Same two 20 m worlds, same six movers
and lanes (`../driving_moving_rpp/worlds/`), 4 planners, replanning at 1 or 5 Hz,
3 runs each, 10 goals per run: 48 runs, one at a time, 16 November 2026, 15:07 to
20:05. Script: `../run_moving_odom.sh` (a copy of `../run_moving.sh` with
`LCP_SIM_LOCALIZATION=odometry` and its own output directory).

## Setup

As in `../driving_moving_rpp/README.md` (movers 0.3 m/s, Regulated Pure Pursuit,
leg timeout 150 s of sim time, planner CPU = CPU seconds of the `planner_server`
process, which includes the shared global-costmap work), except:

- **Localisation:** `LCP_SIM_LOCALIZATION=odometry`: no AMCL; the map frame is
  fixed at the spawn pose, so the pose is the simulator's wheel odometry. It cannot
  jump, but nothing corrects it if the wheels slip, for example when the robot
  touches a moving cylinder.
- The launch log of the smoke run confirms AMCL was not running.
- Start-up failures, repeated automatically: `open/5hz/lcp_03` (localisation did
  not start). No invalid runs: no run has a goal counted as reached in no time
  (the `sim/drive.py` check added after the AMCL batch was active).

## Results (mean ± sd over 3 runs; `runs_all.csv` has every run)

The last column repeats the AMCL batch's means (goals · recoveries · CPU s).

| World, replanning | Planner | Goals of 10 | Runs with 10/10 | Recoveries | Sim time (s) | Planner CPU (s) | AMCL batch |
|---|---|---|---|---|---|---|---|
| open, 1 Hz | LCP | 9.0 ± 1.7 | 2/3 | 7.7 ± 1.5 | 398 ± 220 | 20.5 ± 10.3 | 10.0 · 6.3 · 15.3 |
|  | NavFn | 10.0 ± 0.0 | 3/3 | 4.3 ± 4.2 | 278 ± 15 | 14.7 ± 2.0 | 10.0 · 1.3 · 14.5 |
|  | Smac 2D | 10.0 ± 0.0 | 3/3 | 10.0 ± 3.0 | 283 ± 9 | 14.8 ± 1.8 | 10.0 · 5.3 · 15.5 |
|  | Theta\* | 9.7 ± 0.6 | 2/3 | 11.0 ± 5.3 | 319 ± 80 | 15.9 ± 4.9 | 10.0 · 6.0 · 15.6 |
| open, 5 Hz | LCP | 8.7 ± 2.3 | 2/3 | 9.0 ± 3.6 | 438 ± 283 | 24.7 ± 17.6 | 10.0 · 5.7 · 14.9 |
|  | NavFn | 10.0 ± 0.0 | 3/3 | 3.7 ± 3.8 | 273 ± 8 | 15.7 ± 1.2 | 10.0 · 4.0 · 17.4 |
|  | Smac 2D | 10.0 ± 0.0 | 3/3 | 10.7 ± 3.5 | 286 ± 8 | 16.0 ± 1.8 | 10.0 · 3.3 · 19.1 |
|  | Theta\* | 9.7 ± 0.6 | 2/3 | 9.7 ± 2.1 | 265 ± 15 | 14.6 ± 0.5 | 10.0 · 11.3 · 16.5 |
| 10 % density, 1 Hz | LCP | 10.0 ± 0.0 | 3/3 | 6.7 ± 4.6 | 340 ± 13 | 17.2 ± 0.6 | 9.7 · 15.7 · 17.5 |
|  | NavFn | 10.0 ± 0.0 | 3/3 | 4.3 ± 1.5 | 346 ± 3 | 17.7 ± 0.7 | 10.0 · 1.7 · 19.6 |
|  | Smac 2D | 10.0 ± 0.0 | 3/3 | 6.3 ± 2.1 | 342 ± 5 | 17.6 ± 2.3 | 9.0 · 7.0 · 17.6 |
|  | Theta\* | 9.7 ± 0.6 | 2/3 | 14.3 ± 7.2 | 352 ± 10 | 18.1 ± 1.9 | 10.0 · 4.3 · 19.7 |
| 10 % density, 5 Hz | LCP | 10.0 ± 0.0 | 3/3 | 5.7 ± 1.5 | 327 ± 1 | 19.2 ± 1.2 | 10.0 · 9.7 · 19.7 |
|  | NavFn | 9.0 ± 1.7 | 2/3 | 17.7 ± 25.6 | 450 ± 163 | 31.5 ± 10.7 | 10.0 · 1.3 · 20.9 |
|  | Smac 2D | 10.0 ± 0.0 | 3/3 | 6.0 ± 0.0 | 344 ± 1 | 22.3 ± 1.9 | 9.3 · 8.0 · 20.4 |
|  | Theta\* | 7.7 ± 1.2 | 0/3 | 40.3 ± 17.8 | 433 ± 72 | 28.7 ± 4.2 | 9.0 · 5.7 · 20.0 |

Runs that missed at least one goal, out of 12 per planner:

| Planner | Odometry | AMCL |
|---|---|---|
| LCP | 2 | 1 |
| NavFn | 1 | 0 |
| Smac 2D | 0 | 2 |
| Theta\* | 6 | 3 |
| All | 9 of 48 | 6 of 48 |

Per-condition tables with Mann-Whitney tests: `<world>/<rate>hz/summary.md`. With
3 runs against 3 no p-value can fall below 0.10.

## Findings

- **Switching localisation did not make the test cleaner.** Incomplete runs went
  from 6 to 9 of 48. Odometry removes AMCL's pose jumps but has its own failure
  mode with moving obstacles; the traces record only the believed pose, not the
  simulator's true pose, so how much of it is drift cannot be measured from these
  data. In every bad run the believed pose stayed inside the arena.
- **The planners are not separated on success.** Every planner reaches 9 to 10 of
  10 goals in most conditions; the exceptions are single bad runs (LCP open 5 Hz
  run 1: 6/10; LCP open 1 Hz run 3: 7/10; NavFn 10 % 5 Hz run 3: 7/10; Theta\*
  10 % 5 Hz: 7, 9 and 7). These runs dominate the means and spreads.
- **LCP's two bad runs show a real cost of its fallback.** In both, the planner
  reported "Goal is not reachable from the start" in the open world (no static
  obstacles; the costmap must have held cylinder marks around the goal or robot),
  and once "Final path failed its collision check". Before refusing, LCP's
  weighted A\* fallback searches the whole reachable map, so those plans took
  about 1 s each (the planner server's loop fell to about 1 Hz against its 20 Hz
  target) and the run's planner CPU rose to 32 and 45 s. A grid planner pays that
  cost on every plan; LCP pays it only when it fails, but then it pays all of it.
- **Without the bad runs, CPU follows the static-clutter pattern of the AMCL
  batch:** in the 10 % world at 5 Hz LCP used 19.2 ± 1.2 s, Smac 2D 22.3, NavFn
  31.5 (one bad run), Theta\* 28.7; at 1 Hz all four were within 1 s of each other.
  The open-world 5 Hz CPU advantage seen with AMCL (LCP 14.9 s, lowest) is not
  reproduced here because one of LCP's three runs there was a bad run.
- **In the 10 % world LCP was the most consistent planner with odometry:** 6 of 6
  runs complete, 5.7 to 6.7 recoveries per run (NavFn 4.3 and 17.7; Theta\* 14.3
  and 40.3). With AMCL the same world gave LCP its worst run (32 recoveries); with
  odometry that did not recur, which supports reading that run as a localisation
  effect.
- **Faster replanning still does not make anyone arrive sooner.**

## What the two batches say together

Across 96 runs and two localisation modes, moving cylinders at 0.3 m/s do not
separate the four planners on success, travel time, or (beyond single runs)
recoveries; the outcome of a run is decided by rare navigation failures that hit
every planner. LCP's planning-time advantage, clear in the planner-only benchmark,
appears in driving only as planner-server CPU, and only when it does not fall back.
For the paper: report the test as showing that LCP drives as reliably as the Nav2
grid planners with moving obstacles, not that it drives better; and report the
fallback cost as a limitation.

## Limits

- 3 runs per condition; nothing is statistically significant.
- No ground-truth pose was logged, so odometry drift cannot be separated from
  costmap marking or planning failures. A future run should record the
  simulator's true pose (e.g. Gazebo's pose topic) alongside the believed one.
- One controller, one obstacle speed, six movers, 20 m worlds, simulation only.
- Planner CPU includes the global-costmap updates.
