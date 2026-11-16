| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 8.3 ± 0.6 | 0 | 465.2 ± 16.7 | 32.7 ± 9.1 | 34 | 154.7 ± 4.1 | 0.24 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 528.0 ± 10.5 | 2.3 ± 3.2 | 1 | 191.3 ± 1.8 | 0.24 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 415.7 ± 4.2 | 2.0 ± 3.5 | 0 | 151.2 ± 0.1 | 0.26 |
| theta | 3 | 7.7 ± 0.6 | 0 | 457.8 ± 28.5 | 44.7 ± 4.7 | 43 | 147.8 ± 2.0 | 0.25 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.0593 | 0.1 | 0.1 |
| smac2d | 0.0593 | 0.0765 | 0.1 |
| theta | 0.302 | 0.121 | 0.7 |
