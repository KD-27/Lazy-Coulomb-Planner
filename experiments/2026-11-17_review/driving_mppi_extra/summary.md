| Planner | Runs | Goals reached (of 15), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 5 | 14.8 ± 0.4 | 4 | 210.4 ± 52.2 | 2.4 ± 5.4 | 0 | 47.0 ± 0.6 | 0.22 |
| navfn | 5 | 15.0 ± 0.0 | 5 | 218.1 ± 22.8 | 1.0 ± 2.2 | 0 | 49.2 ± 0.2 | 0.28 |
| smac2d | 5 | 15.0 ± 0.0 | 5 | 190.5 ± 1.0 | 0.0 ± 0.0 | 0 | 47.0 ± 0.2 | 0.22 |
| theta | 5 | 14.8 ± 0.4 | 4 | 203.8 ± 14.5 | 8.0 ± 11.3 | 4 | 48.1 ± 0.4 | 0.23 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.424 | 1 | 0.151 |
| smac2d | 0.424 | 0.424 | 0.675 |
| theta | 1 | 0.181 | 0.222 |
