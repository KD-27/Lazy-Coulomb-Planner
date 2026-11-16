| Planner | Runs | Goals reached (of 15), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 5 | 15.0 ± 0.0 | 5 | 185.0 ± 4.8 | 0.0 ± 0.0 | 0 | 47.1 ± 0.4 | 0.24 |
| navfn | 5 | 15.0 ± 0.0 | 5 | 247.7 ± 37.7 | 4.2 ± 4.0 | 5 | 49.5 ± 0.2 | 0.29 |
| smac2d | 5 | 14.8 ± 0.4 | 4 | 212.5 ± 48.9 | 2.6 ± 5.8 | 0 | 46.7 ± 0.3 | 0.22 |
| theta | 5 | 14.8 ± 0.4 | 4 | 219.1 ± 26.9 | 8.4 ± 11.6 | 3 | 48.0 ± 0.6 | 0.22 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 1 | 0.0248 | 0.00794 |
| smac2d | 0.424 | 0.424 | 0.0556 |
| theta | 0.424 | 0.00729 | 0.00794 |
