| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 10.0 ± 0.0 | 3 | 278.6 ± 5.9 | 6.3 ± 3.8 | 8 | 93.0 ± 1.1 | 0.15 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 277.4 ± 4.8 | 1.3 ± 2.3 | 0 | 93.5 ± 0.7 | 0.13 |
| smac2d | 3 | 10.0 ± 0.0 | 3 | 279.9 ± 2.0 | 5.3 ± 3.2 | 4 | 94.8 ± 0.3 | 0.17 |
| theta | 3 | 10.0 ± 0.0 | 3 | 274.9 ± 5.8 | 6.0 ± 1.7 | 7 | 92.5 ± 0.4 | 0.13 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.184 | 0.7 |
| smac2d | 1 | 1 | 1 |
| theta | 1 | 0.658 | 0.7 |
