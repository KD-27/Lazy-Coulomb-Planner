| Planner | Runs | Goals reached (of 15), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 10.0 ± 0.0 | 3 | 341.7 ± 9.2 | 3.7 ± 6.4 | 0 | 122.9 ± 1.2 | 0.28 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 343.5 ± 1.0 | 0.0 ± 0.0 | 0 | 124.9 ± 0.0 | 0.49 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 341.6 ± 0.4 | 0.0 ± 0.0 | 0 | 124.7 ± 0.1 | 0.40 |
| theta | 3 | 10.0 ± 0.0 | 3 | 338.7 ± 0.5 | 0.0 ± 0.0 | 0 | 123.7 ± 0.0 | 0.37 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.505 | 0.7 |
| smac2d | 1 | 0.505 | 1 |
| theta | 1 | 0.505 | 0.7 |
