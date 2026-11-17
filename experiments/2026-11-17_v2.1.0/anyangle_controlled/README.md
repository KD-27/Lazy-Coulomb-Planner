# Any-angle and grid baselines from outside Nav2: Anya, JPS, Theta\*, Lazy Theta\* (planner v2.1.0)

The literature review (`reports/LCP literature review and gap.md`, outside the repository) names the optimal any-angle planners and jump point search as the baselines a reviewer will ask for. This run times them on exactly the maps and start/goal pairs of the controlled suite, `../../2026-11-15_v2.1.0/controlled/` (21 maps, 100 pairs each), so LCP, NavFn, Smac 2D and Nav2's Theta\* did not have to be run again. Run on 17 November 2026 with the machine otherwise idle.

## Setup

- **Planners:** the public-domain Java collection [Any-Angle-Pathfinding](https://github.com/Ohohcakester/Any-Angle-Pathfinding) (commit 96874d6, 2022-12-20). Its Anya is Daniel Harabor's own implementation (Anya16, optimal any-angle). Also timed: jump point search (JPS, optimal 8-connected), JPS with post-smoothing (`JPS_PS`), Theta\* and Lazy Theta\*. The collection is cloned to `~/lcp_jazzy_ws/baselines/aap` (outside the repository). It is compiled with our runner `benchmark/anyangle/AnyAngleBench.java` (`javac -sourcepath aap/src`; the collection's own `ant` build is not needed).
- **Same obstacles as LCP:** `benchmark/anyangle/export_grid.py` blocks a tile when the Nav2 costmap would give it cost 253 or more: an obstacle or unknown cell, or a cell centre within 0.224 m of an obstacle cell centre, the same radius `benchmark/routability.py` uses. Start and goal are the suite's pairs, rounded to the nearest grid vertex, because these planners work on cell corners. The rounding moves endpoints by up to 3.5 cm, so length ratios carry about 1 % noise.
- **Timing:** each query is run once untimed to warm up the JIT, then 5 timed runs, and the median is reported. Java's `System.nanoTime` measures the search only. LCP's and the Nav2 planners' times are planner_server's own planning time (from the controlled suite, median of 5), which also includes post-processing and building the ROS path message. LCP's floor for that is about 0.07 ms (its time on the empty map). Different language, so the times are indicative.
- `run_anyangle.sh` produced the per-map CSVs. `compare.py` produced `comparison.csv` and `comparison.md` (time, pairs solved, path length relative to Anya, for every planner and map). The blocked grids are in `grids/` (gzipped).
- **First attempt kept:** `../anyangle_controlled_r0.22/` blocked only cells within 0.22 m. That left a slightly more open map than LCP's costmap, so Anya "solved" pairs that have no route for the robot (100 against 94 routable at 30 % density). Its times are within noise of this run.

## Findings

Median planning time in ms:

| map | LCP | NavFn | Smac 2D | Nav2 Theta\* | Anya | JPS | JPS + smoothing | Lazy Theta\* |
|---|---|---|---|---|---|---|---|---|
| empty | **0.07** | 11.1 | 9.3 | 11.2 | 0.19 | 1.57 | 1.50 | 1.36 |
| 2.5 % | 0.16 | 14.1 | 14.4 | 10.7 | **0.13** | 0.26 | 0.21 | 1.91 |
| 5 % | 0.25 | 14.8 | 13.5 | 11.4 | 0.21 | 0.16 | **0.13** | 2.77 |
| 10 % (base) | 0.44 | 14.3 | 15.8 | 13.3 | 0.37 | 0.16 | **0.12** | 3.90 |
| 20 % | 1.26 | 13.8 | 14.4 | 12.3 | 0.34 | 0.13 | **0.10** | 4.27 |
| 30 % | 3.41 | 13.4 | 15.3 | 13.5 | 0.69 | 0.15 | **0.12** | 6.34 |
| 40 % | 5.92 | 13.5 | 15.2 | 8.7 | 0.92 | 0.21 | **0.19** | 7.18 |
| U shapes | 2.34 | 13.0 | 17.6 | 16.4 | 0.32 | 0.16 | **0.11** | 7.70 |
| corridor 0.8 m | 1.59 | 4.2 | 6.5 | 13.1 | 0.25 | 0.13 | **0.10** | 4.79 |
| 10 m map | 0.10 | 0.79 | 1.48 | 0.94 | 0.07 | 0.05 | **0.04** | 0.12 |
| 100 m map | 0.75 | 54.1 | 54.1 | 42.4 | 0.43 | 0.27 | **0.24** | 14.3 |
| 200 m map | 3.03 | 286 | 135 | 221 | 1.79 | 0.85 | **0.82** | 64.1 |

Median path length relative to Anya (optimal any-angle on the same grid):

| map | LCP | NavFn | Smac 2D | Nav2 Theta\* | JPS | JPS + smoothing |
|---|---|---|---|---|---|---|
| empty | 1.000 | 1.004 | 1.021 | 1.003 | 1.060 | 1.000 |
| 10 % (base) | 1.024 | 1.045 | 1.054 | 1.030 | 1.057 | 1.019 |
| 20 % | 1.052 | 1.117 | 1.077 | 1.062 | 1.050 | 1.016 |
| 30 % | 1.087 | 1.143 | 1.087 | 1.091 | 1.049 | 1.016 |
| 40 % | 1.101 | 1.144 | 1.092 | 1.097 | 1.046 | 1.014 |
| 200 m map | 1.024 | 1.033 | 1.052 | 1.023 | 1.053 | 1.021 |

- **LCP is not faster than the state of the art outside Nav2.** JPS with post-smoothing is faster than LCP on every map except the empty one and the 2.5 % one, by 1.9x at 5 % density, 3.7x on the 200 m map and 28x at 30 % density. Anya, which returns optimal paths, is faster than LCP on every map except the empty one, though only by 1.2x at 2.5 % and 5 %. Even on the 200 m map Anya takes 1.8 ms against LCP's 3.0 ms.
- **LCP wins only on nearly empty maps:** 0.07 ms on the empty map against 0.19 ms for Anya and 1.5 ms for JPS, which scans long jumps across open space; and 0.16 ms at 2.5 % against 0.21 ms for JPS with smoothing (Anya 0.13 ms).
- **Against Nav2's planners the speed-up stands:** 10x to 90x on the same pairs (see the controlled suite).
- **Path quality:** LCP's paths are within 2.5 % of optimal on open and large maps and 5 to 10 % longer at 20 to 45 % density. That is similar to Smac 2D and Nav2's Theta\* and better than NavFn. JPS with post-smoothing stays within 2 %.
- **Success:** Anya solves exactly the routable pairs on every map, as LCP does. JPS and the other Java planners return paths for more pairs than are routable on the densest maps (100 against 94 at 30 %, 85 against 71 at 35 %). The collection lets them pass diagonally between two blocked tiles that touch at a corner, which the robot cannot do. Their success counts are therefore not comparable there; their times are.

## What this means for the paper

The paper cannot claim LCP is faster than grid planners in general. The defensible claim is narrower:

- among the planners available to Nav2 users, LCP is one to two orders of magnitude faster on the live costmap, solves every routable pair and never touches the inscribed zone;
- dedicated grid planners such as JPS and Anya, which are not available as Nav2 plugins, are faster still on a binarised grid.

Together with `../ablation_line_then_astar/`, most of LCP's advantage over Nav2 comes from the line check and a fast weighted A\*, not from the Coulomb repair.

## Limits

Java against C++, and search-only time against planner_server time, so the times are indicative, not a head-to-head. No ROS integration of the Java planners, so no clearance pass, smoothing or costmap costs for them. A Nav2 plugin wrapping JPS or Anya would be the fair comparison and does not exist. One seed of maps and pairs.
