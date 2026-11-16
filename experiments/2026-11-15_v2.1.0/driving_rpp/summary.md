| Planner | Runs | Goals reached (of 15), mean ± sd | Runs with all goals | Sim time (s) | Recoveries | Median recoveries | Distance (m) | Closest obstacle (m) |
|---|---|---|---|---|---|---|---|---|
| lcp | 10 | 14.3 ± 0.9 | 5 | 208.0 ± 29.0 | 35.2 ± 21.3 | 37.5 | 41.1 ± 1.7 | 0.22 |
| navfn | 10 | 14.7 ± 0.5 | 7 | 185.8 ± 10.5 | 9.9 ± 6.4 | 10.5 | 43.8 ± 0.3 | 0.24 |
| smac2d | 10 | 11.4 ± 1.3 | 0 | 229.7 ± 17.3 | 67.3 ± 15.6 | 65.5 | 34.9 ± 2.5 | 0.23 |
| theta | 10 | 12.5 ± 0.8 | 0 | 225.9 ± 19.2 | 53.1 ± 15.9 | 56 | 40.7 ± 1.6 | 0.24 |

Mann-Whitney U, two-sided, lcp against each other planner (p-values):

| Against | Goals reached | Recoveries | Sim time |
|---|---|---|---|
| navfn | 0.335 | 0.00344 | 0.104 |
| smac2d | 0.000404 | 0.00282 | 0.0452 |
| theta | 0.00184 | 0.045 | 0.121 |
