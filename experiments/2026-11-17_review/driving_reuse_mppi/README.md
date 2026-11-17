# LCP path reuse on vs off, mppi (planner v2.1.0)

Reviewer request: only LCP keeps its previous path between replans (path_reuse_distance
0.5 m, min saving 15 %), and path stability alone could reduce controller recoveries. LCP
was driven in Nav2's sandbox world, 15 goals per run, MPPI (Docker image `lcp-jazzy:latest`, as in ../../2026-11-15_v2.1.0/driving_mppi/), 5 + 5 runs interleaved
(reuse off, reuse on, reuse off, ...), on 17 November 2026. Reuse off = planner parameter
`path_reuse_distance: 0.0`, set through the new `LCP_SIM_PLANNER_PARAMS` of
`sim/run_sim.sh` and checked in the running planner (`ros2 param get`). Runs:
`reuse_off/lcp_NN`, `reuse_on/lcp_NN` (each with `summary.md`); comparison against
each other and the 15 November runs: `comparison.md` (`../compare_reuse.py`).

## Finding

Reuse off: 15.0 ± 0.0 goals, 1.4 ± 2.6 recoveries, 218 ± 31 s. Reuse on: 14.6 ± 0.5 goals, 8.0 ± 7.4 recoveries, 263 ± 67 s. Recoveries p = 0.27, time p = 0.69 (two-sided Mann-Whitney). **No significant effect of path reuse.** The reuse-on runs did not reproduce the 15 November runs (0.0 ± 0.0 recoveries, 185 ± 5 s): three of five had 12 to 15 recoveries, so five MPPI runs are too few to rank planners; see ../driving_mppi_pooled/.

## Limits

One world, one machine, AMCL localisation; recoveries vary a lot from run to run.
