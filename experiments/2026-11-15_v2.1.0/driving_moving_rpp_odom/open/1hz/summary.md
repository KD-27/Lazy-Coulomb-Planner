| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 9.0 ± 1.7 | 2 | 398.5 ± 220.3 | 7.7 ± 1.5 | 8 | 134.7 ± 76.4 | 0.12 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 277.8 ± 14.8 | 4.3 ± 4.2 | 3 | 91.5 ± 0.1 | 0.12 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 282.7 ± 9.2 | 10.0 ± 3.0 | 10 | 93.8 ± 0.8 | 0.13 |
| theta | 3 | 9.7 ± 0.6 | 2 | 318.6 ± 79.6 | 11.0 ± 5.3 | 9 | 101.7 ± 18.5 | 0.13 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.505 | 0.507 | 0.7 |
| smac2d | 0.505 | 0.4 | 0.7 |
| theta | 1 | 0.507 | 1 |
