| Planner | Runs | Goals reached (of 15), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 10 | 14.9 ± 0.3 | 9 | 197.7 ± 37.4 | 1.2 ± 3.8 | 0 | 47.0 ± 0.5 | 0.23 |
| navfn | 10 | 15.0 ± 0.0 | 10 | 232.9 ± 33.3 | 2.6 ± 3.5 | 0.5 | 49.4 ± 0.2 | 0.29 |
| smac2d | 10 | 14.9 ± 0.3 | 9 | 201.5 ± 34.6 | 1.3 ± 4.1 | 0 | 46.8 ± 0.3 | 0.22 |
| theta | 10 | 14.8 ± 0.4 | 8 | 211.4 ± 21.9 | 8.2 ± 10.8 | 3.5 | 48.0 ± 0.5 | 0.22 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.368 | 0.112 | 0.00281 |
| smac2d | 1 | 1 | 0.0816 |
| theta | 0.583 | 0.00278 | 0.00578 |
