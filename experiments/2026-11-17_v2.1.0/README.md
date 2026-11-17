# Experiments of 17 November 2026 (planner v2.1.0)

Answers to the reviewer threats listed by the literature review: is LCP more than "check the straight line, else A*", how does it compare with the any-angle and grid planners outside Nav2, and where does it lose clearance. Planner `v2.1.0` (unchanged source, optimised build), machine otherwise idle. Chain: `run_chain.sh`, then `run_after.sh`; log `run.log`.

| directory | what was varied | headline | caveats |
|---|---|---|---|
| `ablation_line_then_astar/` | LCP against "straight line, else weighted A*" (weight 3 and 1) and a fixed push side, 21 controlled maps | The line check plus LCP's weighted A* is already 4x to 65x faster than NavFn and Smac 2D. The lazy repair adds 1.3x to 2.7x on sparse and large maps, and costs up to 2.6x from 20 % density. The Coulomb side choice is 1.7x to 2x faster than a fixed side. | one seed |
| `anyangle_controlled/` | Anya, JPS, JPS + smoothing, Theta*, Lazy Theta* (Java collection) on the controlled suite's maps and pairs | JPS + smoothing and Anya are faster than LCP on every map except the nearly empty ones (200 m map: LCP 3.0 ms, Anya 1.8 ms, JPS 0.8 ms). LCP's paths are within 2.5 % of optimal on open maps and 5 to 10 % longer when dense. | Java search-only time against planner_server time; JPS passes diagonal gaps the robot cannot |
| `anyangle_controlled_r0.22/` | same, blocked radius 0.22 m | **superseded** by `anyangle_controlled/` (grid slightly too open) | |
| `ablation_clearance_stages/` | LCP stopped after each post-processing stage, against NavFn, Smac 2D, Theta*, 8 ablation maps | Clearance is lost by the lazy correction itself (segments graze obstacles), not by simplification. The clearance pass restores the median smallest clearance from 0.225 m to 0.356 m (NavFn 0.425, Theta* 0.399, Smac 2D 0.378). | single seed; Constrained Smoother not tested |

New tooling used here, committed on 17 November: clearance columns in `benchmark/lcp_bench.py` and `run_suite.py`, `benchmark/anyangle/` (export, runner, comparison) and the two ablation suites in `benchmark/suites/ablation/`.
