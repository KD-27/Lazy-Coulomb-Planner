| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 8.0 ± 2.0 | 1 | 414.4 ± 27.1 | 50.0 ± 30.2 | 54 | 123.0 ± 21.2 | 0.20 |
| navfn | 3 | 9.3 ± 0.6 | 1 | 399.8 ± 19.4 | 19.7 ± 17.9 | 24 | 133.5 ± 2.9 | 0.24 |
| smac2d | 3 | 4.7 ± 1.5 | 0 | 309.1 ± 120.6 | 60.7 ± 33.3 | 66 | 82.2 ± 30.3 | 0.20 |
| theta | 3 | 5.0 ± 2.6 | 0 | 381.9 ± 71.6 | 76.7 ± 43.5 | 77 | 106.2 ± 44.0 | 0.24 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.5 | 0.4 | 0.7 |
| smac2d | 0.121 | 0.7 | 0.2 |
| theta | 0.268 | 0.7 | 1 |
