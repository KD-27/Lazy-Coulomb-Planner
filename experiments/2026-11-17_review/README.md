# Experiments for the external review (17 November 2026, planner v2.1.0)

An external review of the paper (7 major issues, a list of inconsistencies and missing
details) asked for tuned baselines, a path-reuse ablation, tail latency, a lazy-vs-fallback
path comparison and a fixed-side row. Everything below ran on the same laptop as before,
optimised build, one heavy job at a time. The planner source is unchanged (v2.1.0); the
tooling changed: `sim/run_sim.sh` takes `LCP_SIM_PLANNER_PARAMS`, `benchmark/run_suite.py`
no longer crashes when a planner has no valid path on a map.

Scripts: `run_chain.sh` (tuned baselines, then the two path-reuse driving batches),
`run_after.sh` (side rule), `run_after2.sh` (resume of the tuned baselines after the
summary crash), `run_after3.sh` (extra MPPI runs); `run.log` has the timeline.

| Directory | What | Main result |
|---|---|---|
| `tuned_baselines/` | NavFn with A*, Smac 2D and Theta* with cost weight 0, vs LCP and line-else-A*, 21 controlled maps | NavFn A* is 1.1-4.3x faster than NavFn; LCP still 5-48x faster than it up to 20 % density, 20x/34x at 100/200 m, even at 40 %, 14 % slower at 45 %. Cost-0 Smac/Theta* lose most clean paths. Smac default column invalid (static parameter) |
| `offline/` | From existing data: lazy vs fallback paths, Coulomb vs probe per pair, tail latency | Fallback paths no worse relative to NavFn; probe as good and faster; LCP p95 below NavFn's at every density |
| `ablation_side_rule/` | Coulomb, probe, fixed side in one run, 8 maps | 489 / 449 / 440 lazy solves; 1.21 / 0.81 / 1.70 ms |
| `replay_unsafe/` | Replay of 2 fixed-side paths that touched cost 253 | Cell-edge rounding in the audit, not reproducible; no planner issue |
| `driving_reuse_rpp/` | LCP path reuse off vs on, RPP, 10+10 runs | No significant effect (recoveries 43 vs 36, p = 0.40) |
| `driving_reuse_mppi/` | The same with MPPI, 5+5 runs | No significant effect (1.4 vs 8.0, p = 0.27); 15 Nov MPPI result not reproduced |
| `driving_mppi_extra/`, `driving_mppi_pooled/` | 5 more MPPI runs per planner, pooled to 10 | LCP 1.2, NavFn 2.6, Smac 2D 1.3, Theta* 8.2 recoveries; only Theta* separates |

Paper changes made from these results: reframing around the line check plus weighted A*,
new Section "Against tuned grid planners" (Table), tail columns in Table I, side-rule block in
Table II, MPPI column of Table IV replaced by the 10-run pool, path-reuse paragraph, LCP
presented with two side rules (Coulomb default, probe faster).
