# Side rule on the 21 controlled maps, planner v2.1.0

## Obstacle density

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle density | Routable | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|---|
| 0 % | 100 | 100 · 11.48 ms | 100 · 0.07 ms | 100 · 0.06 ms | 100 · 0.06 ms | 100 · 0.07 ms |
| 2.5 % | 100 | 100 · 14.72 ms | 100 · 0.17 ms | 100 · 0.14 ms | 99 · 0.17 ms | 99 · 0.15 ms |
| 5 % | 100 | 100 · 15.87 ms | 100 · 0.25 ms | 100 · 0.19 ms | 99 · 0.26 ms | 97 · 0.20 ms |
| 10 % | 100 | 100 · 15.02 ms | 100 · 0.44 ms | 100 · 0.35 ms | 97 · 0.43 ms | 93 · 0.34 ms |
| 15 % | 100 | 99 · 16.01 ms | 100 · 0.62 ms | 100 · 0.41 ms | 89 · 0.57 ms | 89 · 0.35 ms |
| 20 % | 100 | 94 · 14.29 ms | 100 · 1.28 ms | 100 · 0.71 ms | 74 · 0.69 ms | 75 · 0.49 ms |
| 25 % | 100 | 81 · 13.04 ms | 100 · 2.58 ms | 100 · 1.35 ms | 45 · 0.88 ms | 55 · 0.53 ms |
| 30 % | 94 | 84 · 14.33 ms | 94 · 3.56 ms | 94 · 2.14 ms | 35 · 0.98 ms | 26 · 0.46 ms |
| 35 % | 71 | 57 · 16.65 ms | 71 · 4.50 ms | 71 · 2.63 ms | 14 · 1.57 ms | 13 · 0.46 ms |
| 40 % | 74 | 53 · 19.29 ms | 74 · 6.00 ms | 74 · 3.66 ms | 12 · 0.88 ms | 13 · 0.73 ms |
| 45 % | 30 | 15 · 15.72 ms | 30 · 6.02 ms | 30 · 3.29 ms | 4 · 1.20 ms | 2 · 0.97 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle density | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| 0 % | - | - | - | - | - |
| 2.5 % | - | - | - | 0 touching, 1 failed | 0 touching, 1 failed |
| 5 % | - | - | - | 0 touching, 1 failed | 0 touching, 3 failed |
| 10 % | - | - | - | 0 touching, 3 failed | 0 touching, 7 failed |
| 15 % | 1 touching, 0 failed | - | - | 0 touching, 11 failed | 0 touching, 11 failed |
| 20 % | 6 touching, 0 failed | - | - | 0 touching, 26 failed | 0 touching, 25 failed |
| 25 % | 19 touching, 0 failed | - | - | 0 touching, 55 failed | 0 touching, 45 failed |
| 30 % | 10 touching, 0 failed | - | - | 0 touching, 59 failed | 0 touching, 68 failed |
| 35 % | 14 touching, 0 failed | - | - | 0 touching, 57 failed | 0 touching, 58 failed |
| 40 % | 21 touching, 0 failed | - | - | 0 touching, 62 failed | 0 touching, 61 failed |
| 45 % | 15 touching, 0 failed | - | - | 0 touching, 26 failed | 0 touching, 28 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle density | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| 0 % | 19.9 · 1.000 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 |
| 2.5 % | 26.8 · 1.000 · 0.0 | 3.6 · 0.994 · 1.9 | 1.2 · 0.994 · 1.7 | 0.7 · 0.994 · 1.8 | 0.6 · 0.994 · 1.7 |
| 5 % | 27.5 · 1.000 · 0.0 | 4.2 · 0.992 · 6.3 | 1.7 · 0.992 · 6.7 | 2.4 · 0.992 · 6.1 | 0.6 · 0.992 · 6.5 |
| 10 % | 27.9 · 1.000 · 0.4 | 3.9 · 0.983 · 16.2 | 2.2 · 0.985 · 17.5 | 2.6 · 0.982 · 16.2 | 0.9 · 0.983 · 17.2 |
| 15 % | 27.4 · 1.000 · 2.1 | 3.8 · 0.975 · 32.6 | 2.3 · 0.971 · 32.8 | 2.9 · 0.974 · 30.5 | 1.5 · 0.967 · 32.2 |
| 20 % | 27.6 · 1.000 · 7.7 | 7.5 · 0.959 · 48.6 | 5.2 · 0.949 · 48.5 | 3.1 · 0.964 · 45.9 | 1.9 · 0.955 · 46.0 |
| 25 % | 27.4 · 1.000 · 15.5 | 10.6 · 0.936 · 60.4 | 10.0 · 0.936 · 59.6 | 2.7 · 0.935 · 55.7 | 1.8 · 0.934 · 56.0 |
| 30 % | 30.4 · 1.000 · 29.6 | 8.8 · 0.951 · 72.8 | 7.3 · 0.950 · 73.2 | 3.6 · 0.952 · 71.6 | 1.6 · 0.951 · 73.2 |
| 35 % | 32.9 · 1.000 · 43.5 | 11.0 · 0.928 · 91.7 | 13.0 · 0.926 · 90.7 | 3.1 · 0.961 · 89.0 | 1.7 · 0.933 · 78.8 |
| 40 % | 97.5 · 1.000 · 68.1 | 17.5 · 0.956 · 101.5 | 13.5 · 0.953 · 101.5 | 3.3 · 0.945 · 84.3 | 1.5 · 0.944 · 92.8 |
| 45 % | 67.7 · 1.000 · 86.5 | 16.3 · 0.938 · 113.8 | 11.1 · 0.934 · 113.8 | 1.6 · 0.949 · 96.7 | 1.2 · 0.943 · 123.0 |

## Map size

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Map size | Routable | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|---|
| 10 m | 100 | 100 · 0.76 ms | 100 · 0.10 ms | 100 · 0.08 ms | 100 · 0.10 ms | 100 · 0.08 ms |
| 25 m | 100 | 100 · 4.29 ms | 100 · 0.21 ms | 100 · 0.15 ms | 98 · 0.19 ms | 97 · 0.15 ms |
| 50 m | 100 | 100 · 15.02 ms | 100 · 0.44 ms | 100 · 0.35 ms | 97 · 0.43 ms | 93 · 0.34 ms |
| 100 m | 100 | 100 · 55.00 ms | 100 · 0.77 ms | 100 · 0.54 ms | 98 · 0.76 ms | 96 · 0.50 ms |
| 200 m | 100 | 100 · 303.85 ms | 100 · 3.56 ms | 100 · 3.09 ms | 73 · 2.75 ms | 65 · 2.32 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Map size | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| 10 m | - | - | - | - | - |
| 25 m | - | - | - | 0 touching, 2 failed | 0 touching, 3 failed |
| 50 m | - | - | - | 0 touching, 3 failed | 0 touching, 7 failed |
| 100 m | - | - | - | 0 touching, 2 failed | 0 touching, 4 failed |
| 200 m | - | - | - | 0 touching, 27 failed | 0 touching, 35 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Map size | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| 10 m | 1.2 · 1.000 · 4.5 | 0.8 · 0.969 · 20.6 | 0.4 · 0.971 · 20.6 | 0.6 · 0.969 · 20.6 | 0.3 · 0.971 · 20.6 |
| 25 m | 8.1 · 1.000 · 1.2 | 3.1 · 0.982 · 18.1 | 1.2 · 0.980 · 16.5 | 1.3 · 0.983 · 17.9 | 0.8 · 0.980 · 16.2 |
| 50 m | 27.9 · 1.000 · 0.4 | 3.9 · 0.983 · 16.2 | 2.2 · 0.985 · 17.5 | 2.6 · 0.982 · 16.2 | 0.9 · 0.983 · 17.2 |
| 100 m | 110.3 · 1.000 · 0.3 | 8.3 · 0.986 · 18.5 | 4.9 · 0.984 · 18.2 | 3.9 · 0.986 · 18.0 | 2.4 · 0.984 · 18.0 |
| 200 m | 536.6 · 1.000 · 0.5 | 13.3 · 0.992 · 23.9 | 14.8 · 0.993 · 22.4 | 4.9 · 0.987 · 21.1 | 3.9 · 0.986 · 20.1 |

## Obstacle shape

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle shape | Routable | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|---|
| square | 100 | 100 · 14.10 ms | 100 · 0.35 ms | 100 · 0.27 ms | 100 · 0.34 ms | 98 · 0.26 ms |
| u | 98 | 93 · 14.03 ms | 98 · 2.94 ms | 98 · 1.77 ms | 43 · 0.57 ms | 30 · 0.18 ms |
| circle | 100 | 100 · 15.02 ms | 100 · 0.44 ms | 100 · 0.35 ms | 97 · 0.43 ms | 93 · 0.34 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle shape | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| square | - | - | - | - | 0 touching, 2 failed |
| u | 5 touching, 0 failed | - | - | 0 touching, 55 failed | 0 touching, 68 failed |
| circle | - | - | - | 0 touching, 3 failed | 0 touching, 7 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle shape | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| square | 27.2 · 1.000 · 0.6 | 2.2 · 0.977 · 19.4 | 1.6 · 0.973 · 19.6 | 2.1 · 0.977 · 19.4 | 1.4 · 0.973 · 19.6 |
| u | 27.6 · 1.000 · 5.4 | 7.7 · 0.974 · 42.8 | 5.7 · 0.976 · 42.0 | 3.0 · 0.987 · 30.8 | 1.6 · 0.986 · 16.8 |
| circle | 27.9 · 1.000 · 0.4 | 3.9 · 0.983 · 16.2 | 2.2 · 0.985 · 17.5 | 2.6 · 0.982 · 16.2 | 0.9 · 0.983 · 17.2 |

## Corridor width

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Corridor width | Routable | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|---|
| 0.8 m | 100 | 99 · 4.50 ms | 100 · 1.60 ms | 100 · 1.06 ms | 64 · 1.29 ms | 62 · 0.46 ms |
| 1.2 m | 100 | 100 · 7.51 ms | 100 · 0.64 ms | 100 · 0.40 ms | 87 · 0.61 ms | 87 · 0.33 ms |
| 2 m | 100 | 100 · 7.80 ms | 100 · 0.54 ms | 100 · 0.31 ms | 97 · 0.49 ms | 97 · 0.30 ms |
| 3 m | 100 | 100 · 9.58 ms | 100 · 0.36 ms | 100 · 0.23 ms | 100 · 0.34 ms | 100 · 0.23 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Corridor width | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| 0.8 m | 1 touching, 0 failed | - | - | 0 touching, 36 failed | 0 touching, 38 failed |
| 1.2 m | - | - | - | 0 touching, 13 failed | 0 touching, 13 failed |
| 2 m | - | - | - | 0 touching, 3 failed | 0 touching, 3 failed |
| 3 m | - | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Corridor width | Navfn | coulomb | probe | coulomb_lazy | probe_lazy |
|---|---|---|---|---|---|
| 0.8 m | 9.1 · 1.000 · 147.7 | 3.1 · 1.007 · 147.2 | 2.9 · 1.006 · 147.3 | 2.3 · 1.006 · 147.2 | 1.3 · 1.004 · 147.7 |
| 1.2 m | 18.0 · 1.000 · 74.3 | 2.9 · 0.975 · 82.7 | 2.5 · 0.976 · 82.0 | 2.1 · 0.976 · 83.1 | 1.6 · 0.977 · 83.0 |
| 2 m | 15.2 · 1.000 · 0.7 | 3.2 · 0.976 · 23.3 | 2.6 · 0.976 · 23.5 | 2.2 · 0.977 · 23.3 | 1.7 · 0.977 · 23.5 |
| 3 m | 17.5 · 1.000 · 0.3 | 1.9 · 0.983 · 12.7 | 1.5 · 0.983 · 12.8 | 1.8 · 0.983 · 12.7 | 1.5 · 0.983 · 12.8 |

