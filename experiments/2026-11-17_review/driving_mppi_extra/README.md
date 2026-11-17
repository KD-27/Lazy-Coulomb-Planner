# Five more MPPI runs per planner (planner v2.1.0)

Because the MPPI results of 15 November (5 runs per planner) did not reproduce for LCP
(`../driving_reuse_mppi/`), every planner was driven 5 more times on 17 November 2026,
interleaved (lcp, navfn, smac2d, theta, lcp, ...), same setup as
`../../2026-11-15_v2.1.0/driving_mppi/` (sandbox world, 15 goals, MPPI in Docker, all
defaults, LCP with path reuse on). Runs are numbered 06-10 and pooled with the earlier
01-05 in `../driving_mppi_pooled/` (symbolic links; `summary.md` there).

## These five runs

| Planner | Goals | Recoveries | Sim time (s) |
|---|---|---|---|
| LCP | 14.8 ± 0.4 | 2.4 ± 5.4 | 210 ± 52 |
| NavFn | 15.0 ± 0.0 | 1.0 ± 2.2 | 218 ± 23 |
| Smac 2D | 15.0 ± 0.0 | 0.0 ± 0.0 | 191 ± 1 |
| Theta* | 14.8 ± 0.4 | 8.0 ± 11.3 | 204 ± 15 |

## Pooled, 10 runs per planner (`../driving_mppi_pooled/summary.md`)

| Planner | Goals | Recoveries | Sim time (s) |
|---|---|---|---|
| LCP | 14.9 ± 0.3 | 1.2 ± 3.8 | 198 ± 37 |
| NavFn | 15.0 ± 0.0 | 2.6 ± 3.5 | 233 ± 33 |
| Smac 2D | 14.9 ± 0.3 | 1.3 ± 4.1 | 202 ± 35 |
| Theta* | 14.8 ± 0.4 | 8.2 ± 10.8 | 211 ± 22 |

LCP against each (Mann-Whitney, Holm-corrected over three): recoveries vs Theta* p = 0.008,
vs NavFn 0.22, vs Smac 2D 1.0; time vs NavFn p = 0.008, vs Theta* 0.012, vs Smac 2D 0.08.
With the five LCP-only reuse-on runs of `../driving_reuse_mppi/` added, LCP with defaults
averages 3.5 recoveries over 15 MPPI runs, against 2.6 for NavFn: LCP, NavFn and Smac 2D
cannot be ranked on recoveries with MPPI; Theta* has the most. The earlier claim "LCP
reached every goal without a single recovery" (5 runs) does not hold.
