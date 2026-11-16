| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 10.0 ± 0.0 | 3 | 275.1 ± 3.6 | 5.7 ± 2.5 | 6 | 92.6 ± 0.9 | 0.19 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 285.1 ± 11.3 | 4.0 ± 0.0 | 4 | 94.6 ± 1.6 | 0.13 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 278.5 ± 0.6 | 3.3 ± 0.6 | 3 | 94.8 ± 0.4 | 0.22 |
| theta | 3 | 10.0 ± 0.0 | 3 | 282.8 ± 5.8 | 11.3 ± 5.0 | 12 | 93.2 ± 0.9 | 0.14 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.643 | 0.7 |
| smac2d | 1 | 0.354 | 0.7 |
| theta | 1 | 0.268 | 0.1 |
