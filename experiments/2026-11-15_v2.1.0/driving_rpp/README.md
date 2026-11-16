# Driving test, Regulated Pure Pursuit, planner v2.1.0

A simulated TurtleBot3 drives the same 15 goals in Nav2's sandbox world (nine
pillars in a hexagonal arena), with the full Nav2 stack (AMCL, behaviour tree,
recovery behaviours) and Regulated Pure Pursuit as the path follower. Only the
global planner differs. 10 runs per planner, one at a time on an otherwise idle
machine, 15 November 2026, 18:36 to 21:22. LCP is `v2.1.0`, optimised.
Summary by `sim/summarise_runs.py` (`summary.md`, `runs.csv`).

| Planner | Goals reached of 15 | Runs with all 15 | Sim time (s) | Recoveries per run | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|
| NavFn | 14.7 ± 0.5 | 7 | 186 ± 11 | 9.9 ± 6.4 | 43.8 | 0.24 |
| LCP | 14.3 ± 0.9 | 5 | 208 ± 29 | 35.2 ± 21.3 | 41.1 | 0.22 |
| Theta\* | 12.5 ± 0.8 | 0 | 226 ± 19 | 53.1 ± 15.9 | 40.7 | 0.24 |
| Smac 2D | 11.4 ± 1.3 | 0 | 230 ± 17 | 67.3 ± 15.6 | 34.9 | 0.23 |

Mann-Whitney U, LCP against NavFn: goals p = 0.34, recoveries p = 0.003, time
p = 0.10. Against Theta\*: goals p = 0.002, recoveries p = 0.045. Against
Smac 2D: goals p = 0.0004, recoveries p = 0.003.

## Findings

- LCP reaches as many goals as NavFn (no significant difference) and
  significantly more than Theta\* and Smac 2D, with fewer recoveries than both.
- NavFn still triggers clearly fewer recovery behaviours (10 against 35 per run).
  This controller stops whenever the path ahead runs within about 0.3 m of an
  obstacle, and NavFn's cost-following paths keep to the middle of the gaps
  between pillars.
- Against v2.0.0 (`../../2026-11-15/driving_rpp/`, same setup) LCP is unchanged
  within noise: 14.3 against 14.1 goals, 35 against 39 recoveries (p = 0.68).

## Limits

One world, one controller, 10 runs. One run of the MPPI batch that followed was
disturbed at its end by an edit to `sim/run_sim.sh` (see `../driving_mppi/`); no
run here was affected.
