# Controlled benchmark repeated over random seeds (planner v2.0.0)

`../controlled_v2.0.0/` generated each map configuration once (seed 1). This
suite repeats the density, size and concave-shape configurations with seeds 2
to 5, so the results can be given with their spread. 36 more maps, same
protocol: 100 pairs per map, every pair planned 5 times by five planners,
ground-truth routability. Run 15 November 2026, 07:27 to 08:20.

`all_seeds.md` and `all_seeds.png` combine both suites: 5 maps per density
(1 at 0 %, where there is nothing to randomise), 3 per size (5 at 50 m, 1 at
200 m) and 3 with U shapes. Produced by `benchmark/aggregate_seeds.py`.

## Results (mean ± standard deviation over seeds)

## Obstacle density (50 m map, random circles)

| Density | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | Theta* clean (%) |
|---|---|---|---|---|---|---|---|---|---|
| 0 % | 1 | 0.18 ± 0.00 | 9.61 ± 0.00 | 52.8 ± 0.0× | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 2.5 % | 5 | 0.56 ± 0.01 | 10.31 ± 0.37 | 18.4 ± 0.8× | 100.0 ± 0.0 | 99.8 ± 0.4 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 5 % | 5 | 0.79 ± 0.12 | 10.85 ± 1.93 | 13.7 ± 1.0× | 100.0 ± 0.0 | 99.6 ± 0.5 | 100.0 ± 0.0 | 100.0 ± 0.0 | 99.8 ± 0.4 |
| 10 % | 5 | 1.25 ± 0.10 | 12.13 ± 1.21 | 9.7 ± 0.5× | 100.0 ± 0.0 | 98.8 ± 1.1 | 99.2 ± 0.8 | 99.8 ± 0.4 | 99.8 ± 0.4 |
| 15 % | 5 | 1.82 ± 0.18 | 13.01 ± 1.63 | 7.1 ± 0.6× | 100.0 ± 0.0 | 94.6 ± 4.3 | 99.4 ± 0.5 | 98.0 ± 1.2 | 98.6 ± 1.1 |
| 20 % | 5 | 2.92 ± 0.39 | 13.66 ± 0.96 | 4.7 ± 0.8× | 100.0 ± 0.0 | 85.5 ± 3.7 | 96.2 ± 1.6 | 93.9 ± 3.6 | 95.1 ± 6.2 |
| 25 % | 5 | 6.53 ± 1.35 | 14.33 ± 0.95 | 2.3 ± 0.7× | 100.0 ± 0.0 | 74.0 ± 2.6 | 90.7 ± 5.7 | 87.3 ± 6.4 | 87.3 ± 6.4 |
| 30 % | 5 | 14.60 ± 2.18 | 15.11 ± 1.93 | 1.1 ± 0.3× | 100.0 ± 0.0 | 56.5 ± 2.6 | 85.8 ± 5.1 | 85.5 ± 4.5 | 84.7 ± 8.7 |

## Map size (10 % density)

| Size | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | Theta* clean (%) |
|---|---|---|---|---|---|---|---|---|---|
| 10 m | 3 | 0.25 ± 0.02 | 0.73 ± 0.05 | 3.0 ± 0.2× | 100.0 ± 0.0 | 99.7 ± 0.6 | 99.7 ± 0.6 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 25 m | 3 | 0.67 ± 0.05 | 3.55 ± 0.27 | 5.4 ± 0.5× | 100.0 ± 0.0 | 99.3 ± 0.6 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 50 m | 5 | 1.25 ± 0.10 | 12.13 ± 1.21 | 9.7 ± 0.5× | 100.0 ± 0.0 | 98.8 ± 1.1 | 99.2 ± 0.8 | 99.8 ± 0.4 | 99.8 ± 0.4 |
| 100 m | 3 | 2.38 ± 0.07 | 45.23 ± 3.33 | 19.0 ± 1.1× | 100.0 ± 0.0 | 97.3 ± 1.5 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 |
| 200 m | 1 | 4.46 ± 0.00 | 135.32 ± 0.00 | 30.3 ± 0.0× | 100.0 ± 0.0 | 97.0 ± 0.0 | 100.0 ± 0.0 | 58.0 ± 0.0 | 100.0 ± 0.0 |

## Concave U-shaped obstacles (50 m, 10 %)

| Shape | Maps | LCP median (ms) | Fastest grid planner (ms) | LCP faster by | LCP solved (%) | Lazy search alone (%) | NavFn clean (%) | Smac 2D clean (%) | Theta* clean (%) |
|---|---|---|---|---|---|---|---|---|---|
| U shapes | 3 | 12.88 ± 9.06 | 13.39 ± 0.12 | 1.7 ± 1.5× | 100.0 ± 0.0 | 56.5 ± 9.5 | 97.2 ± 2.6 | 93.4 ± 1.7 | 98.6 ± 0.6 |

## Findings

- **The open-space advantage holds across seeds, with small spread.** On a 50 m
  map LCP is 18 ± 1 times faster than the fastest grid planner at 2.5 % density,
  10 ± 0.5 times at 10 % and 4.7 ± 0.8 times at 20 %.
- **Crossover at about 30 %.** At 25 % LCP is still 2.3 ± 0.7 times faster; at
  30 % the two are level (1.1 ± 0.3). On this kind of map a grid planner becomes
  the better choice at roughly 30 % obstacle density.
- **LCP solved every routable pair on all 54 maps of the two suites with a clean
  path** (5,365 of 5,365 routable pairs), and correctly refused every pair that
  has no route.
- **The grid planners lose clean paths as density rises**: at 25 % density NavFn
  91 ± 6 %, Smac 2D and Theta\* 87 ± 6 %; at 30 % all three about 85 %. The rest
  are paths that pass through cells where the robot would touch an obstacle.
- **The lazy search alone degrades smoothly**: 99 % at 10 %, 95 ± 4 % at 15 %,
  86 ± 4 % at 20 %, 74 ± 3 % at 25 %, 57 ± 3 % at 30 %. The fallback makes up the
  difference and is the reason LCP's time climbs.
- **Size scaling is confirmed**: 3.0 ± 0.2 times faster at 10 m, 5.4 ± 0.5 at
  25 m, 9.7 ± 0.5 at 50 m, 19 ± 1 at 100 m, 30 at 200 m (one map).
- **Concave obstacles are LCP's hard case.** With U shapes the lazy search alone
  solves 57 ± 10 %, and LCP's median time varies a lot between seeds
  (12.9 ± 9.1 ms), averaging no better than the grid planners.

## Limits

- 5 seeds per density is enough for a mean and a standard deviation, not for
  tight confidence intervals. The 0 % and 200 m points are single maps.
- Corridor widths and square obstacles were not repeated.
- All planners at Nav2 default settings; one costmap configuration (robot radius
  0.22 m, inflation radius 0.7 m).
