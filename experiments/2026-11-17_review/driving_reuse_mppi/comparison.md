# MPPI, LCP path reuse on vs off (sandbox, 15 goals)

| Set | Runs | reached | recoveries | sim_time_s | travelled_m | closest_obstacle_m |
|---|---|---|---|---|---|---|
| reuse off (new) | 5 | 15.0 ± 0.0 | 1.4 ± 2.6 | 217.9 ± 31.1 | 47.7 ± 0.5 | 0.2 ± 0.0 |
| reuse on (new) | 5 | 14.6 ± 0.5 | 8.0 ± 7.4 | 263.3 ± 66.5 | 46.9 ± 1.1 | 0.2 ± 0.0 |
| reuse on (2026-11-15) | 5 | 15.0 ± 0.0 | 0.0 ± 0.0 | 185.0 ± 4.8 | 47.1 ± 0.4 | 0.2 ± 0.0 |

Mann-Whitney U, two-sided, reuse off (new) against:

| Against | reached | recoveries | sim_time_s | travelled_m | closest_obstacle_m |
|---|---|---|---|---|---|
| reuse on (new) | 0.177 | 0.265 | 0.69 | 0.222 | 0.116 |
| reuse on (2026-11-15) | n/a | 0.18 | 0.00794 | 0.0952 | 0.0157 |
