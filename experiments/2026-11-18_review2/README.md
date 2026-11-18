# Experiments for the second review (18 November 2026, planner v2.1.0)

A second review of the paper (another AI model, verdict weak accept / accept) asked, as its item 2,
for the Coulomb-vs-probe comparison on the full controlled suite rather than only the eight
ablation maps. Planner source unchanged (v2.1.0, optimised build, same install as
`../2026-11-17_review/`). Script: `run.sh`; timeline in `run.log` (10:50 to 11:07, machine idle).

| Directory | What | Main result |
|---|---|---|
| `side_rule_controlled/` | Coulomb (default) and probe side rule, grid fallback on and off (`*_lazy`), plus NavFn; the 21 controlled maps incl. 100/200 m; 100 pairs, 3 repeats, one run (all variants on the same pairs). Suite `benchmark/suites/ablation/side_rule_controlled.yaml` | Fallback on: both rules solve all 1967 routable pairs with clean paths and refuse all 133 without a route; probe 1.08-1.91x faster (median 1.5), same length and cost. Fallback off: Coulomb 1528, probe 1494 of 1967; Coulomb more on 11 maps, probe on 3, 7 ties |

## side_rule_controlled: per map

Median time with the fallback on (ms), and pairs solved by the lazy search alone (clean, of routable).

| Map | Coulomb | Probe | Coulomb / probe | Lazy only, Coulomb | Lazy only, probe |
|---|---|---|---|---|---|
| density 0 % | 0.068 | 0.063 | 1.08 | 100 | 100 |
| 2.5 % | 0.165 | 0.145 | 1.14 | 99 | 99 |
| 5 % | 0.252 | 0.194 | 1.30 | 99 | 97 |
| 10 % | 0.440 | 0.354 | 1.24 | 97 | 93 |
| 15 % | 0.617 | 0.412 | 1.50 | 89 | 89 |
| 20 % | 1.281 | 0.707 | 1.81 | 74 | 75 |
| 25 % | 2.585 | 1.352 | 1.91 | 45 | 55 |
| 30 % | 3.565 | 2.139 | 1.67 | 35 | 26 |
| 35 % | 4.505 | 2.626 | 1.72 | 14 | 13 |
| 40 % | 5.997 | 3.664 | 1.64 | 12 | 13 |
| 45 % | 6.023 | 3.294 | 1.83 | 4 | 2 |
| 10 m | 0.095 | 0.079 | 1.20 | 100 | 100 |
| 25 m | 0.211 | 0.148 | 1.43 | 98 | 97 |
| 100 m | 0.765 | 0.542 | 1.41 | 98 | 96 |
| 200 m | 3.560 | 3.093 | 1.15 | 73 | 65 |
| square obstacles | 0.352 | 0.268 | 1.31 | 100 | 98 |
| U obstacles | 2.938 | 1.771 | 1.66 | 43 | 30 |
| corridor 0.8 m | 1.599 | 1.065 | 1.50 | 64 | 62 |
| corridor 1.2 m | 0.642 | 0.404 | 1.59 | 87 | 87 |
| corridor 2.0 m | 0.539 | 0.306 | 1.76 | 97 | 97 |
| corridor 3.0 m | 0.360 | 0.226 | 1.59 | 100 | 100 |

- Path length relative to NavFn and mean costmap cost differ by at most about 1 % and 1.6 cost units
  per map between the two rules (`summary.csv`).
- The probe's p95 is lower on 20 of 21 maps (10 % density: 1.48 against 1.47 ms).
- Reading: the probe is the faster rule whenever the fallback is available. Coulomb repairs more
  requests by itself (+34 of 1967), mostly on U-shaped obstacles (+13), 30 % density (+9) and the
  200 m map (+8); at 25 % the probe repairs more (+10). This repeats the 8-map ablation
  (`../2026-11-17_review/ablation_side_rule/`), now on the whole controlled suite.
- `progress.log` counts "solved" against each planner's own result; use `summary_truth.csv` /
  `routability.txt` (ground-truth routability) for the numbers above.
