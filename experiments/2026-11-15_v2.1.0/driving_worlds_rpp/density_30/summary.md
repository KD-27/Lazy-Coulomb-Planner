| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 6.0 ± 1.7 | 0 | 366.3 ± 48.8 | 87.3 ± 51.7 | 72 | 82.9 ± 21.4 | 0.20 |
| navfn | 3 | 6.7 ± 0.6 | 0 | 382.3 ± 73.2 | 61.7 ± 5.0 | 61 | 108.0 ± 30.5 | 0.20 |
| smac2d | 3 | 6.3 ± 1.2 | 0 | 390.4 ± 24.0 | 61.0 ± 23.9 | 56 | 110.4 ± 20.1 | 0.22 |
| theta | 3 | 6.7 ± 1.2 | 0 | 363.7 ± 17.8 | 56.7 ± 7.4 | 54 | 100.4 ± 9.0 | 0.20 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.7 | 1 |
| smac2d | 1 | 0.7 | 1 |
| theta | 1 | 0.7 | 0.7 |
