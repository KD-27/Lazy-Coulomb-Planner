# Tuned grid baselines (planner v2.1.0)

Reviewer request: NavFn, Smac 2D and Theta* optimise a cost that includes clearance,
which weakens their heuristics; do they lose only because of that objective? The 21 maps
of the controlled suite (same generator, seeds and pairs as `../../2026-11-15_v2.1.0/controlled/`),
100 pairs, 5 repeats (median), ground-truth routability. Run on 17 November 2026,
optimised build, machine otherwise idle. Suite: `benchmark/suites/ablation/tuned_baselines.yaml`.

Variants: `Navfn` (default), `Navfn_astar` (`use_astar: true`; NavFn has no cost weight),
`Smac2d` / `Smac2d_nocost` (`cost_travel_multiplier: 0`), `ThetaStar` / `ThetaStar_nocost`
(`w_traversal_cost: 0`), `LCP` (default), `line_astar_w3` (LCP with `max_iterations: 0`).

The run stopped after 9 maps on a bug in `run_suite.py` (a planner with no valid path on a
map: `ThetaStar_nocost` at 35 %); fixed (NaN instead of a crash) and resumed after the
driving runs, finished maps kept (`run_after2.sh`).

## Invalid column: Smac2d (default)

`Node2D::cost_travel_multiplier` is a **static** member in Nav2 Jazzy
(`/opt/ros/jazzy/include/nav2_smac_planner/node_2d.hpp:271`), so the two Smac 2D instances in
one planner server share it and the last one configured (0) wins. The `Smac2d` column here is
therefore Smac 2D with the cost term off as well (its numbers match `Smac2d_nocost`). Default
Smac 2D on the same maps and pairs: `../../2026-11-17_v2.1.0/ablation_line_then_astar/`
(and `../../2026-11-15_v2.1.0/controlled/`). Theta* keeps its weight per instance.

## Findings

Median planning time (ms); speed-up of LCP and of "line, else A*" over NavFn with A*:

| map | NavFn | NavFn A* | Smac 2D cost 0 | Theta* | Theta* cost 0 | LCP | line, else A* | LCP / NavFn A* | line / NavFn A* |
|---|---|---|---|---|---|---|---|---|---|
| empty | 10.8 | 3.28 | 9.79 | 11.3 | 8.46 | 0.07 | 0.06 | 48x | 52x |
| 2.5 % | 15.0 | 3.51 | 14.2 | 11.3 | 11.9 | 0.15 | 0.44 | 24x | 8.0x |
| 5 % | 16.3 | 4.06 | 12.8 | 12.5 | 13.7 | 0.25 | 0.57 | 16x | 7.1x |
| 10 % | 14.0 | 4.71 | 11.5 | 14.0 | 12.7 | 0.43 | 0.70 | 11x | 6.7x |
| 15 % | 16.0 | 5.80 | 9.38 | 16.1 | 12.0 | 0.65 | 0.82 | 8.9x | 7.1x |
| 20 % | 15.5 | 6.09 | 8.98 | 14.0 | 11.2 | 1.18 | 0.95 | 5.1x | 6.4x |
| 25 % | 12.8 | 5.89 | 8.31 | 12.7 | 12.7 | 2.89 | 1.13 | 2.0x | 5.2x |
| 30 % | 13.9 | 5.85 | 9.93 | 15.5 | 14.2 | 3.71 | 1.43 | 1.6x | 4.1x |
| 35 % | 16.7 | 6.06 | 10.6 | 17.9 | 15.6 | 4.01 | 1.74 | 1.5x | 3.5x |
| 40 % | 21.7 | 6.08 | 9.88 | 16.7 | 14.5 | 5.99 | 3.06 | 1.0x | 2.0x |
| 45 % | 17.0 | 5.76 | 7.65 | 12.2 | 9.74 | 6.67 | 3.49 | 0.86x | 1.7x |
| 10 m | 0.77 | 0.34 | 0.93 | 0.95 | 0.50 | 0.10 | 0.13 | 3.5x | 2.7x |
| 25 m | 4.08 | 1.31 | 3.97 | 3.96 | 3.08 | 0.21 | 0.28 | 6.3x | 4.7x |
| 100 m | 54.3 | 16.6 | 48.5 | 46.9 | 49.9 | 0.84 | 2.20 | 20x | 7.6x |
| 200 m | 330 | 103 | 116 | 234 | 206 | 3.05 | 4.87 | 34x | 21x |
| squares | 13.7 | 4.19 | 7.44 | 10.6 | 9.51 | 0.35 | 0.66 | 12x | 6.4x |
| U shapes | 13.5 | 5.56 | 12.5 | 15.5 | 22.1 | 2.71 | 0.98 | 2.1x | 5.7x |
| corridor 0.8 m | 4.17 | 3.82 | 4.56 | 13.1 | 10.5 | 1.55 | 0.81 | 2.5x | 4.7x |
| corridor 1.2 m | 7.39 | 4.74 | 5.66 | 12.5 | 13.5 | 0.70 | 0.80 | 6.8x | 5.9x |
| corridor 2.0 m | 7.50 | 4.19 | 9.78 | 13.8 | 22.6 | 0.56 | 0.83 | 7.5x | 5.1x |
| corridor 3.0 m | 8.85 | 3.80 | 8.56 | 11.5 | 13.6 | 0.35 | 0.56 | 11x | 6.7x |

Share of the 1967 routable pairs solved with a path that stays out of the inscribed zone
(cost < 253): LCP 100 %, line-else-A* 100 %, NavFn 95.3 %, NavFn A* 94.1 %, Theta* 93.0 %,
Theta* cost 0 24.7 %, Smac 2D cost 0 47.9 %.

- **NavFn with A\* is the meaningful tuned baseline:** 1.1x to 4.3x faster than default NavFn
  (about 3x on most maps) with the same share of clean paths.
- **LCP stays ahead of it on sparse, large and corridor maps** (5x to 48x up to 20 % density,
  20x at 100 m, 34x at 200 m), is 1.5x to 2x faster at 25 to 35 %, even at 40 % and 14 %
  slower at 45 %. "Line, else A*" is 1.7x to 52x faster than NavFn A* on every map.
- **Switching off the cost term speeds Smac 2D and Theta\* up somewhat, at the price of
  safety.** Smac 2D with cost 0 is faster than default Smac 2D (same day, `ablation_line_then_astar`)
  on 19 of 21 maps, median 1.5x, at most 2.6x; Theta* with cost 0 is faster than Theta* on 14
  maps (median 1.1x, from 0.6x to 1.9x). Both stay slower than NavFn with A*, which is the
  fastest tuned grid planner on all 21 maps. And their paths graze obstacles: only a quarter of
  Theta*'s and half of Smac 2D's paths stay out of the inscribed zone (Theta* default: 93 %).

## Limits

One map per configuration, one set of pairs, one machine. NavFn has no cost weight to switch off;
`use_astar` is its only search option. Smac 2D's default column could not be measured in this
run (static parameter, above).
