| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 8.7 ± 2.3 | 2 | 437.8 ± 283.0 | 9.0 ± 3.6 | 8 | 146.2 ± 93.7 | 0.12 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 273.4 ± 8.2 | 3.7 ± 3.8 | 2 | 91.6 ± 0.4 | 0.17 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 285.7 ± 8.4 | 10.7 ± 3.5 | 11 | 93.1 ± 0.4 | 0.12 |
| theta | 3 | 9.7 ± 0.6 | 2 | 264.6 ± 15.0 | 9.7 ± 2.1 | 9 | 87.0 ± 5.9 | 0.12 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.505 | 0.268 | 0.4 |
| smac2d | 0.505 | 0.7 | 0.7 |
| theta | 1 | 0.825 | 0.4 |
