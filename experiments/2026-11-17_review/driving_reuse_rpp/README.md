# LCP path reuse on vs off, rpp (planner v2.1.0)

Reviewer request: only LCP keeps its previous path between replans (path_reuse_distance
0.5 m, min saving 15 %), and path stability alone could reduce controller recoveries. LCP
was driven in Nav2's sandbox world, 15 goals per run, Regulated Pure Pursuit (native, as in ../../2026-11-15_v2.1.0/driving_rpp/), 10 + 10 runs interleaved
(reuse off, reuse on, reuse off, ...), on 17 November 2026. Reuse off = planner parameter
`path_reuse_distance: 0.0`, set through the new `LCP_SIM_PLANNER_PARAMS` of
`sim/run_sim.sh` and checked in the running planner (`ros2 param get`). Runs:
`reuse_off/lcp_NN`, `reuse_on/lcp_NN` (each with `summary.md`); comparison against
each other and the 15 November runs: `comparison.md` (`../compare_reuse.py`).

## Finding

Reuse off: 14.2 ± 1.0 goals, 43 ± 18 recoveries, 224 ± 23 s. Reuse on: 14.4 ± 0.7 goals, 36 ± 18 recoveries, 212 ± 24 s. Recoveries p = 0.40, goals p = 0.84, time p = 0.21. **No significant effect of path reuse.** The reuse-on runs reproduce the 15 November runs closely (35 ± 21 recoveries, 208 ± 29 s).

## Limits

One world, one machine, AMCL localisation; recoveries vary a lot from run to run.
