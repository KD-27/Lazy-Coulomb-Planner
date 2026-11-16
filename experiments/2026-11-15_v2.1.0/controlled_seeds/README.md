# Controlled benchmark over random seeds, planner v2.1.0

The density, map-size and concave-shape configurations of `../controlled/`
repeated with seeds 2 to 5 (48 more maps), same protocol: 100 pairs per map,
every pair planned 5 times by five planners, ground-truth routability. Run
15 November 2026, 16:37 to 17:39, on `v2.1.0` compiled optimised.

`all_seeds.md` and `all_seeds.png` combine both suites: 5 maps per density
(1 at 0 %), 3 per size (5 at 50 m, 1 at 200 m) and 3 with U shapes.

## Results (mean ± standard deviation over seeds)

## Obstacle density (50 m map, random circles)

| Density | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | Theta* clean (%) |
|---|---|---|---|---|---|---|---|---|---|
| 0 % | 1 | 0.07 ± 0.00 | 9.35 ± 0.00 | 139.5 ± 0.0× | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 2.5 % | 5 | 0.16 ± 0.01 | 10.42 ± 0.43 | 66.2 ± 3.7× | 100.0 ± 0.0 | 99.2 ± 0.8 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 5 % | 5 | 0.23 ± 0.03 | 10.82 ± 1.63 | 46.7 ± 3.7× | 100.0 ± 0.0 | 98.8 ± 1.1 | 100.0 ± 0.0 | 100.0 ± 0.0 | 99.8 ± 0.4 |
| 10 % | 5 | 0.39 ± 0.05 | 11.75 ± 1.16 | 30.6 ± 3.1× | 100.0 ± 0.0 | 96.0 ± 1.9 | 99.2 ± 0.8 | 99.8 ± 0.4 | 99.8 ± 0.4 |
| 15 % | 5 | 0.69 ± 0.09 | 12.90 ± 1.60 | 18.9 ± 2.3× | 100.0 ± 0.0 | 89.4 ± 6.7 | 99.4 ± 0.5 | 98.0 ± 1.2 | 98.6 ± 1.1 |
| 20 % | 5 | 1.53 ± 0.36 | 13.54 ± 0.89 | 9.2 ± 2.2× | 100.0 ± 0.0 | 71.2 ± 4.8 | 96.2 ± 1.6 | 93.9 ± 3.6 | 95.1 ± 6.2 |
| 25 % | 5 | 2.90 ± 0.17 | 14.48 ± 1.29 | 5.0 ± 0.7× | 100.0 ± 0.0 | 48.8 ± 4.4 | 90.7 ± 5.7 | 87.3 ± 6.4 | 87.3 ± 6.4 |
| 30 % | 5 | 3.63 ± 0.25 | 15.01 ± 1.88 | 4.1 ± 0.5× | 100.0 ± 0.0 | 36.5 ± 1.4 | 85.8 ± 5.1 | 85.5 ± 4.5 | 84.7 ± 8.7 |
| 35 % | 5 | 5.37 ± 0.83 | 17.31 ± 4.02 | 3.3 ± 0.7× | 100.0 ± 0.0 | 16.9 ± 2.4 | 69.9 ± 12.2 | 75.5 ± 16.5 | 66.4 ± 13.4 |
| 40 % | 5 | 5.76 ± 1.16 | 14.90 ± 4.03 | 2.6 ± 0.4× | 100.0 ± 0.0 | 17.0 ± 4.5 | 61.0 ± 17.5 | 61.3 ± 20.9 | 57.9 ± 12.9 |
| 45 % | 5 | 4.55 ± 0.85 | 6.53 ± 2.35 | 1.5 ± 0.4× | 100.0 ± 0.0 | 28.8 ± 10.6 | 67.5 ± 16.2 | 79.1 ± 24.4 | 74.7 ± 26.5 |

## Map size (10 % density)

| Size | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | Theta* clean (%) |
|---|---|---|---|---|---|---|---|---|---|
| 10 m | 3 | 0.09 ± 0.01 | 0.73 ± 0.06 | 8.2 ± 0.4× | 100.0 ± 0.0 | 99.3 ± 0.6 | 99.7 ± 0.6 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 25 m | 3 | 0.22 ± 0.04 | 3.54 ± 0.32 | 16.4 ± 2.8× | 100.0 ± 0.0 | 98.7 ± 0.6 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 50 m | 5 | 0.39 ± 0.05 | 11.75 ± 1.16 | 30.6 ± 3.1× | 100.0 ± 0.0 | 96.0 ± 1.9 | 99.2 ± 0.8 | 99.8 ± 0.4 | 99.8 ± 0.4 |
| 100 m | 3 | 0.81 ± 0.05 | 45.47 ± 3.46 | 56.3 ± 2.0× | 100.0 ± 0.0 | 93.7 ± 4.5 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 200 m | 1 | 3.03 ± 0.00 | 135.03 ± 0.00 | 44.5 ± 0.0× | 100.0 ± 0.0 | 73.0 ± 0.0 | 100.0 ± 0.0 | 58.0 ± 0.0 | 100.0 ± 0.0 |

## Concave U-shaped obstacles (50 m, 10 %)

| Shape | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | Theta* clean (%) |
|---|---|---|---|---|---|---|---|---|---|
| U shapes | 3 | 2.46 ± 0.58 | 13.56 ± 0.45 | 5.7 ± 1.4× | 100.0 ± 0.0 | 45.1 ± 10.7 | 97.2 ± 2.6 | 93.4 ± 1.7 | 98.6 ± 0.6 |

## Findings

- **LCP is faster than every grid planner at every density tested.** On a 50 m
  map: 66 ± 4 times faster than the fastest grid planner at 2.5 % obstacle
  density, 31 ± 3 at 10 %, 9 ± 2 at 20 %, 4.1 ± 0.5 at 30 %, 2.6 ± 0.4 at 40 %
  and 1.5 ± 0.4 at 45 %. There is no crossover within the range where random
  maps stay connected; beyond about 45 % almost no pair has a route.
- **The advantage grows with map size**: 8 ± 0.4 times at 10 m, 16 ± 3 at 25 m,
  31 ± 3 at 50 m, 56 ± 2 at 100 m. At 200 m (one map) it is 45, measured
  against Smac 2D, which fails 42 % of the pairs there; against NavFn and
  Theta\*, which solve them all, it is 73 to 95.
- **LCP solved every routable pair on all 69 maps with a clean path** (6,234 of
  6,234) and refused every pair that has no route (666 of 666).
- **The grid planners lose clean paths as density rises.** At 30 % density about
  85 % of their paths are clean, at 40 % about 60 %. Over all maps NavFn
  returned 439 paths that pass through a cell where the robot would touch an
  obstacle, Smac 2D 499 and Theta\* 492. Smac 2D also failed 42 routable pairs.
- **The lazy search alone covers the easy cases and the fallback the rest.** It
  solves 99 % of pairs at 2.5 % density, 96 % at 10 %, 71 % at 20 %, 37 % at
  30 % and 17 % at 40 %. LCP's time rises with density because more requests go
  to the A\* fallback, but it stays below the grid planners' throughout.
- **Concave obstacles** are no longer a weak case for speed: 2.5 ± 0.6 ms against
  13.6 ms, 5.7 ± 1.4 times faster, although the lazy search alone solves only
  45 ± 11 % there.
- **Worst case.** The slowest LCP plan over all 69 maps took 28 ms; reporting
  that a pair has no route took at most 63 ms.
- **Sharp corners.** 133 of 5,783 LCP paths (2.3 %) have a corner sharper than
  90°.

## Notes on the scoring

Smac 2D returned a clean-looking path for 49 pairs that ground truth says have
no route, all on dense maps. Ground truth only allows moves between cells that
share an edge; these paths slip diagonally between two blocked cells that touch
at a corner. LCP and NavFn refuse the same pairs.

## Limits

- Five seeds per density give a mean and a standard deviation, not tight
  confidence intervals. The 0 % and 200 m points are single maps.
- At 45 % density only 9 to 40 of the 100 pairs per map have a route, so that
  row rests on few pairs.
- Corridor widths and square obstacles were not repeated over seeds.
- All planners at Nav2 default settings; one costmap configuration; one machine.
