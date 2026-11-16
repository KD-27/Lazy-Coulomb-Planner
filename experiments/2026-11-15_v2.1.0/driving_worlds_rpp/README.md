# Driving test in generated worlds, Regulated Pure Pursuit, planner v2.1.0

The TurtleBot3 driving test in six Gazebo worlds built from generated maps, the
driving counterpart of the planner benchmark: an open arena, random cylinders at
10, 20 and 30 % obstacle density, concave U-shaped obstacles, and 4 m blocks with
1.2 m corridors. All worlds are 20 m square. 3 runs per world and planner, 10
goals per run, one run at a time, 15 to 16 November 2026, 23:52 to 07:36.

## Setup

- Worlds from `sim/make_worlds.py sim/worlds/density_set.yaml` (in `worlds/`):
  `benchmark/generate_map.py --sdf` writes each world next to its map, circles as
  cylinders and everything else as boxes; the obstacles match the map's occupied
  cells exactly. The robot starts at the free cell nearest the centre with at
  least 0.6 m of room.
- Full Nav2 stack and Regulated Pure Pursuit as in `../driving_rpp/`, except
  **localisation**: the robot's pose is the simulator's wheel odometry (a fixed
  map-to-odom transform at the spawn pose, no AMCL). A first attempt with AMCL
  was stopped after 9 runs because AMCL mislocalised the robot among identical
  cylinders, jumping its estimate 1.5 m into an obstacle, after which every plan
  was refused; those runs are kept in `../driving_worlds_rpp_amcl_aborted/`.
  These results therefore measure planning and path following with reliable
  localisation.
- Goals are drawn with a fixed seed from cells at least 0.3 m from obstacles in
  the start's free region; leg timeout 150 s of simulated time.
- One run (`shape_u/theta_02`) failed to start localisation on the first attempt
  and was repeated automatically.

## Results (mean ± sd over 3 runs; `<world>/summary.md` has the tests)

| World | LCP goals | NavFn | Smac 2D | Theta\* | LCP recoveries | NavFn | Smac 2D | Theta\* |
|---|---|---|---|---|---|---|---|---|
| Open | 10.0 ± 0 | 10.0 ± 0 | 10.0 ± 0 | 10.0 ± 0 | 0 | 0 | 0 | 0 |
| 10 % density | 10.0 ± 0 | 10.0 ± 0 | 10.0 ± 0 | 10.0 ± 0 | 0 | 0 | 0 | 0 |
| 20 % density | 8.3 ± 0.6 | 10.0 ± 0 | 10.0 ± 0 | 7.7 ± 0.6 | 33 | 2 | 2 | 45 |
| 30 % density | 6.0 ± 1.7 | 6.7 ± 0.6 | 6.3 ± 1.2 | 6.7 ± 1.2 | 87 | 62 | 61 | 57 |
| U shapes | 10.0 ± 0 | 10.0 ± 0 | 10.0 ± 0 | 10.0 ± 0 | 4 | 0 | 0 | 0 |
| 1.2 m corridors | 8.0 ± 2.0 | 9.3 ± 0.6 | 4.7 ± 1.5 | 5.0 ± 2.6 | 50 | 20 | 61 | 77 |

Simulated time for the 10 goals in the open world: LCP 256 s, NavFn 257 s,
Smac 2D 266 s, Theta\* 257 s; at 10 % density 319, 328, 331 and 328 s.

## Findings

- In open space and at 10 % density every planner drives every goal without a
  single recovery, and LCP is the quickest by a small margin (shorter paths).
- With concave U-shaped obstacles all four reach every goal.
- At 20 % density and in the 1.2 m corridors the picture of the sandbox world
  returns: NavFn and Smac 2D (at 20 %) drive cleanly, LCP and Theta\*, whose
  paths run closer to obstacles, trigger many recoveries and miss some goals.
  In the corridors LCP (8.0) does better than Smac 2D (4.7) and Theta\* (5.0) and
  worse than NavFn (9.3).
- At 30 % density every planner struggles (6 to 7 of 10 goals); the differences
  are within the spread of three runs.

## Limits

- 3 runs per world and planner: the spread is large in the dense worlds and
  corridors, and most differences there are not significant.
- One controller (Regulated Pure Pursuit); with MPPI LCP drove best in the
  sandbox world (`../driving_mppi/`).
- Wheel-odometry localisation, 20 m worlds, one seed per world, simulation only.
