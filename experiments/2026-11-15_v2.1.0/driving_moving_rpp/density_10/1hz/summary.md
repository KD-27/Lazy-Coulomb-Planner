| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 9.7 ± 0.6 | 2 | 354.9 ± 54.5 | 15.7 ± 14.2 | 9 | 117.6 ± 13.7 | 0.11 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 347.3 ± 1.3 | 1.7 ± 2.1 | 1 | 123.2 ± 0.0 | 0.13 |
| smac2d | 3 | 9.0 ± 1.7 | 2 | 321.8 ± 50.9 | 7.0 ± 3.6 | 6 | 111.3 ± 18.8 | 0.12 |
| theta | 3 | 10.0 ± 0.0 | 3 | 352.7 ± 9.5 | 4.3 ± 4.0 | 2 | 122.6 ± 0.4 | 0.12 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.505 | 0.1 | 0.7 |
| smac2d | 1 | 0.507 | 0.4 |
| theta | 0.505 | 0.261 | 1 |
