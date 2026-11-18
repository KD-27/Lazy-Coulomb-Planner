# Experiments for the second review (18 November 2026, planner v2.1.0)

A second review of the paper (another AI model, verdict weak accept / accept) asked, as its item 2,
for the Coulomb-vs-probe comparison on the full controlled suite rather than only the eight
ablation maps. Planner source unchanged (v2.1.0, optimised build, same install as
`../2026-11-17_review/`). Script: `run.sh`; timeline in `run.log` (10:50 to 11:07, machine idle).

| Directory | What | Main result |
|---|---|---|
| `side_rule_controlled/` | Coulomb (default) and probe side rule, grid fallback on and off (`*_lazy`), plus NavFn; the 21 controlled maps incl. 100/200 m; 100 pairs, 3 repeats, one run (all variants on the same pairs). Suite `benchmark/suites/ablation/side_rule_controlled.yaml` | Fallback on: both rules solve all 1967 routable pairs with clean paths and refuse all 133 without a route; probe 1.08-1.91x faster (median 1.5), same length and cost. Fallback off: Coulomb 1528, probe 1494 of 1967; Coulomb more on 11 maps, probe on 3, 7 ties |
| `r2_controlled/` | R2 (Lai et al., public C++ code github.com/LaiYanKai/2D, commit 04e20b0, Release build) on the blocked grids and vertex queries of `../2026-11-17_v2.1.0/anyangle_controlled/` (same obstacles and pairs as Anya/JPS, blocked radius 0.224 m); 21 maps, 100 pairs, 5 repeats. Scripts `benchmark/anyangle/{export_movingai.py,run_r2.sh,run_r2_safe.py,compare_r2.py}` | R2 is the fastest planner on 15 of 21 maps (JPS+PS at 20-45 % density) and faster than LCP on all 21 (1.4x at 45 %, 4-96x elsewhere); solves exactly the pairs Anya solves, with Anya's optimal length (ratio <= 1.00001). It did not return on 12 of the 133 queries without a route (dropped after 20 s, counted as no path) |

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

## r2_controlled: R2 on the controlled maps

Why: the related work names R2/R2+ (Lai et al.) as the closest vector-based prior work, and its
authors publish C++ code, so a reviewer will ask why it was not run. R2+ and RayScan have no public code.

Setup. `export_movingai.py` turns each blocked grid and query list that `run_anyangle.sh` exported for
Anya and JPS into a MovingAI `.map`/`.map.scen` pair (dots in names replaced by `p`: R2 derives file
names with `replace_extension`). The exported `maps/` (36 MB) are not committed: `run_r2.sh` rebuilds them exactly from the
gzipped grids in `../2026-11-17_v2.1.0/anyangle_controlled/grids/`. `run_r2.sh` runs R2 through `run_r2_safe.py`, 5 repeats per map; the code
times only `alg->run` with `high_resolution_clock`. Per pair the time is the median of the 5 repeats,
per map the median over pairs (`compare_r2.py` -> `r2_summary.csv`). LCP, Anya and JPS+PS columns are the
medians from `../2026-11-17_v2.1.0/anyangle_controlled/comparison.csv` (LCP: C++ Nav2 plugin incl.
post-processing and ROS message, about 0.07 ms; Anya, JPS: Java, search only). Machine idle, 20:17-20:21.

Hang guard. R2 does not return on some queries whose goal is unreachable (first seen at 12:17: the run
stopped inside density_30 query 47 and was left; a 2-minute rerun of that map confirmed it hangs).
`run_r2_safe.py` runs each map once under a 20 s timeout, drops the query that hung, and repeats until a
pass finishes; then the timed 5-repeat run uses the remaining queries, and dropped ones are written as
`timeout` lines (ids in `results/<map>.timeouts.txt`). Dropped: density_30 3 (47, 80, 86), density_35 7,
density_40 2; all 12 are pairs Anya and the routability check say have no route. On the other 121
queries without a route R2 returns "no path" in well under a millisecond. No query with a route hung.

| Map | R2 | LCP | Anya | JPS+PS | Solved R2 / Anya | Timeouts |
|---|---|---|---|---|---|---|
| base | 0.0493 | 0.441 | 0.365 | 0.117 | 100 / 100 | 0 |
| corridor_0.8 | 0.0756 | 1.585 | 0.247 | 0.104 | 100 / 100 | 0 |
| corridor_1.2 | 0.0360 | 0.676 | 0.222 | 0.119 | 100 / 100 | 0 |
| corridor_2.0 | 0.0332 | 0.527 | 0.229 | 0.21 | 100 / 100 | 0 |
| corridor_3.0 | 0.0155 | 0.335 | 0.209 | 0.183 | 100 / 100 | 0 |
| density_00 | 0.0007 | 0.067 | 0.194 | 1.495 | 100 / 100 | 0 |
| density_02 | 0.0066 | 0.159 | 0.131 | 0.211 | 100 / 100 | 0 |
| density_05 | 0.0192 | 0.25 | 0.214 | 0.132 | 100 / 100 | 0 |
| density_15 | 0.0729 | 0.648 | 0.353 | 0.12 | 100 / 100 | 0 |
| density_20 | 0.1384 | 1.264 | 0.342 | 0.103 | 100 / 100 | 0 |
| density_25 | 0.2217 | 2.991 | 0.366 | 0.081 | 100 / 100 | 0 |
| density_30 | 0.3113 | 3.414 | 0.686 | 0.118 | 94 / 94 | 3 |
| density_35 | 0.6899 | 4.379 | 0.699 | 0.178 | 71 / 71 | 7 |
| density_40 | 0.9879 | 5.921 | 0.923 | 0.186 | 74 / 74 | 2 |
| density_45 | 3.6511 | 5.166 | 0.437 | 0.115 | 30 / 30 | 0 |
| shape_square | 0.0176 | 0.346 | 0.189 | 0.094 | 100 / 100 | 0 |
| shape_u | 0.0583 | 2.336 | 0.323 | 0.105 | 98 / 98 | 0 |
| size_010 | 0.0055 | 0.099 | 0.07 | 0.035 | 100 / 100 | 0 |
| size_025 | 0.0187 | 0.207 | 0.126 | 0.056 | 100 / 100 | 0 |
| size_100 | 0.1289 | 0.75 | 0.426 | 0.236 | 100 / 100 | 0 |
| size_200 | 0.7151 | 3.032 | 1.789 | 0.82 | 100 / 100 | 0 |

Median times in ms. R2's path length equals Anya's on every solved pair (median ratio 1.0, max 1.00001).

Findings.
- R2 solves exactly the pairs Anya solves (no mismatch on any map) and returns optimal paths.
- R2 is faster than LCP on every map: 96x on the empty map, 13-24x at 2.5-5 %, 9x at 10-20 %,
  11-14x at 25-30 %, 6x at 35-40 %, 1.4x at 45 %, 16-22x in corridors, 20x and 40x on the square and
  U shapes, 11-18x on the 10 and 25 m maps, 6x on the 100 m map and 4x on the 200 m map.
- R2 is faster than Anya up to 30 % density and on all size, shape and corridor maps (1.6-277x), on par
  at 35-40 %, and 8x slower at 45 %. JPS+PS is the fastest planner at 20 % density and above; R2 is
  the fastest everywhere else (15 of 21 maps).
- So LCP's speed advantage holds only against the Nav2 grid planners. Its remaining case is practical:
  it plans on the inflated costmap, keeps a clearance margin, and is a Nav2 plugin.

Limits. Different languages and timing scopes (R2 C++ search only; LCP includes post-processing and the
ROS message); binary grid at the 0.224 m blocked radius, not the costmap; one machine; the 20 s guard
turns a non-terminating query into a failure, which is how a planner server with a timeout would see it.
