# RPP, LCP path reuse on vs off (sandbox, 15 goals)

| Set | Runs | reached | recoveries | sim_time_s | travelled_m | closest_obstacle_m |
|---|---|---|---|---|---|---|
| reuse off (new) | 10 | 14.2 ± 1.0 | 42.7 ± 17.9 | 224.2 ± 22.7 | 41.3 ± 1.2 | 0.2 ± 0.0 |
| reuse on (new) | 10 | 14.4 ± 0.7 | 35.7 ± 17.4 | 212.3 ± 23.9 | 41.6 ± 0.6 | 0.2 ± 0.0 |
| reuse on (2026-11-15) | 10 | 14.3 ± 0.9 | 35.2 ± 21.3 | 208.0 ± 29.0 | 41.1 ± 1.7 | 0.2 ± 0.0 |

Mann-Whitney U, two-sided, reuse off (new) against:

| Against | reached | recoveries | sim_time_s | travelled_m | closest_obstacle_m |
|---|---|---|---|---|---|
| reuse on (new) | 0.836 | 0.404 | 0.212 | 0.734 | 0.762 |
| reuse on (2026-11-15) | 0.901 | 0.427 | 0.186 | 0.97 | 0.52 |
