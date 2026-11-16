| Planner | Runs | Goals reached (of 10), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 3 | 10.0 ± 0.0 | 3 | 346.1 ± 12.9 | 9.7 ± 6.7 | 8 | 120.9 ± 0.9 | 0.12 |
| navfn | 3 | 10.0 ± 0.0 | 3 | 346.1 ± 2.1 | 1.3 ± 1.5 | 1 | 122.9 ± 0.3 | 0.16 |
| smac2d | 3 | 9.3 ± 1.2 | 2 | 338.4 ± 10.4 | 8.0 ± 6.2 | 6 | 117.3 ± 8.9 | 0.12 |
| theta | 3 | 9.0 ± 0.0 | 0 | 318.5 ± 20.3 | 5.7 ± 4.5 | 6 | 109.2 ± 4.9 | 0.12 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.1 | 0.7 |
| smac2d | 0.505 | 0.7 | 1 |
| theta | 0.0469 | 0.7 | 0.4 |
