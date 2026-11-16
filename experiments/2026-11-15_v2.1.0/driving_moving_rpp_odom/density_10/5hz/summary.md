| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 10.0 ± 0.0 | 3 | 327.3 ± 1.3 | 5.7 ± 1.5 | 6 | 117.9 ± 0.4 | 0.12 |
| navfn | 3 | 9.0 ± 1.7 | 2 | 450.3 ± 163.4 | 17.7 ± 25.6 | 6 | 144.6 ± 34.4 | 0.14 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 344.0 ± 1.0 | 6.0 ± 0.0 | 6 | 120.8 ± 0.5 | 0.16 |
| theta | 3 | 7.7 ± 1.2 | 0 | 433.0 ± 71.9 | 40.3 ± 17.8 | 48 | 129.8 ± 8.9 | 0.12 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.505 | 1 | 0.1 |
| smac2d | 1 | 1 | 0.1 |
| theta | 0.0593 | 0.1 | 0.1 |
