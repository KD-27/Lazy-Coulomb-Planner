| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 10.0 ± 0.0 | 3 | 340.4 ± 13.2 | 6.7 ± 4.6 | 4 | 119.7 ± 3.5 | 0.12 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 346.3 ± 3.1 | 4.3 ± 1.5 | 4 | 120.9 ± 0.8 | 0.14 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 342.2 ± 5.3 | 6.3 ± 2.1 | 7 | 121.6 ± 0.4 | 0.12 |
| theta | 3 | 9.7 ± 0.6 | 2 | 352.1 ± 10.1 | 14.3 ± 7.2 | 18 | 120.2 ± 0.9 | 0.13 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.643 | 0.7 |
| smac2d | 1 | 1 | 1 |
| theta | 0.505 | 0.184 | 0.4 |
