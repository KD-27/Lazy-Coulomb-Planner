# Controlled benchmark, planner v2.1.0

## Obstacle density

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle density | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| 0 % | 100 | 100 · 11.06 ms | 100 · 9.35 ms | 100 · 11.24 ms | 100 · 0.07 ms | 100 · 0.06 ms |
| 2.5 % | 100 | 100 · 14.14 ms | 100 · 14.42 ms | 100 · 10.66 ms | 100 · 0.16 ms | 99 · 0.15 ms |
| 5 % | 100 | 100 · 14.80 ms | 100 · 13.55 ms | 99 · 12.09 ms | 100 · 0.25 ms | 99 · 0.25 ms |
| 10 % | 100 | 100 · 14.33 ms | 100 · 15.81 ms | 100 · 13.28 ms | 100 · 0.44 ms | 98 · 0.43 ms |
| 15 % | 100 | 99 · 15.68 ms | 99 · 17.93 ms | 99 · 14.78 ms | 100 · 0.65 ms | 89 · 0.58 ms |
| 20 % | 100 | 94 · 14.26 ms | 95 · 15.31 ms | 95 · 13.01 ms | 100 · 1.26 ms | 74 · 0.76 ms |
| 25 % | 100 | 81 · 13.10 ms | 80 · 14.95 ms | 83 · 12.79 ms | 100 · 2.99 ms | 45 · 0.92 ms |
| 30 % | 94 | 84 · 13.53 ms | 73 · 18.27 ms | 70 · 14.90 ms | 94 · 3.41 ms | 35 · 0.98 ms |
| 35 % | 71 | 57 · 16.27 ms | 42 · 20.04 ms | 38 · 17.35 ms | 71 · 4.38 ms | 14 · 1.47 ms |
| 40 % | 74 | 52 · 18.02 ms | 57 · 18.03 ms | 38 · 15.73 ms | 74 · 5.92 ms | 12 · 0.87 ms |
| 45 % | 30 | 15 · 14.26 ms | 13 · 10.51 ms | 10 · 10.78 ms | 30 · 5.17 ms | 4 · 1.57 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle density | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0 % | - | - | - | - | - |
| 2.5 % | - | - | - | - | 0 touching, 1 failed |
| 5 % | - | - | 1 touching, 0 failed | - | 0 touching, 1 failed |
| 10 % | - | - | - | - | 0 touching, 2 failed |
| 15 % | 1 touching, 0 failed | 1 touching, 0 failed | 1 touching, 0 failed | - | 0 touching, 11 failed |
| 20 % | 6 touching, 0 failed | 5 touching, 0 failed | 5 touching, 0 failed | - | 0 touching, 26 failed |
| 25 % | 19 touching, 0 failed | 20 touching, 0 failed | 17 touching, 0 failed | - | 0 touching, 55 failed |
| 30 % | 10 touching, 0 failed | 21 touching, 0 failed | 24 touching, 0 failed | - | 0 touching, 59 failed |
| 35 % | 14 touching, 0 failed | 29 touching, 0 failed | 33 touching, 0 failed | - | 0 touching, 57 failed |
| 40 % | 22 touching, 0 failed | 17 touching, 0 failed | 36 touching, 0 failed | - | 0 touching, 62 failed |
| 45 % | 15 touching, 0 failed | 17 touching, 0 failed | 20 touching, 0 failed | - | 0 touching, 26 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle density | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0 % | 19.5 · 1.000 · 0.0 | 85.9 · 1.014 · 0.0 | 63.1 · 1.000 · 0.0 | 0.1 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 |
| 2.5 % | 25.9 · 1.000 · 0.0 | 88.6 · 1.017 · 0.8 | 47.9 · 0.999 · 0.2 | 3.1 · 0.994 · 1.9 | 0.6 · 0.994 · 1.8 |
| 5 % | 27.7 · 1.000 · 0.0 | 77.9 · 1.013 · 2.8 | 58.6 · 0.997 · 1.0 | 3.0 · 0.992 · 6.3 | 1.9 · 0.992 · 6.1 |
| 10 % | 27.5 · 1.000 · 0.4 | 72.9 · 1.004 · 6.4 | 54.8 · 0.990 · 3.2 | 2.6 · 0.983 · 16.2 | 2.6 · 0.982 · 16.2 |
| 15 % | 26.0 · 1.000 · 2.1 | 80.2 · 0.994 · 12.3 | 51.3 · 0.985 · 7.3 | 5.1 · 0.975 · 32.6 | 2.8 · 0.974 · 30.5 |
| 20 % | 26.2 · 1.000 · 7.7 | 111.4 · 0.977 · 21.5 | 85.7 · 0.970 · 15.1 | 5.7 · 0.959 · 48.6 | 2.8 · 0.964 · 45.9 |
| 25 % | 26.0 · 1.000 · 15.5 | 166.2 · 0.958 · 29.7 | 117.6 · 0.962 · 24.9 | 9.4 · 0.936 · 60.4 | 3.3 · 0.935 · 55.7 |
| 30 % | 28.2 · 1.000 · 29.6 | 153.8 · 0.965 · 42.4 | 109.0 · 0.978 · 37.2 | 8.6 · 0.951 · 72.8 | 3.5 · 0.952 · 71.6 |
| 35 % | 28.0 · 1.000 · 43.5 | 110.6 · 0.947 · 60.5 | 81.9 · 0.962 · 56.7 | 11.1 · 0.928 · 91.7 | 2.6 · 0.961 · 89.0 |
| 40 % | 95.3 · 1.000 · 68.2 | 87.6 · 0.961 · 80.5 | 63.5 · 0.975 · 66.0 | 16.1 · 0.956 · 100.0 | 3.3 · 0.945 · 84.3 |
| 45 % | 68.8 · 1.000 · 86.5 | 38.7 · 0.963 · 86.8 | 30.5 · 0.980 · 80.0 | 13.1 · 0.938 · 113.8 | 2.9 · 0.949 · 96.7 |

## Map size

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Map size | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| 10 m | 100 | 100 · 0.79 ms | 100 · 1.48 ms | 100 · 0.94 ms | 100 · 0.10 ms | 100 · 0.10 ms |
| 25 m | 100 | 100 · 4.13 ms | 100 · 4.84 ms | 100 · 3.90 ms | 100 · 0.21 ms | 98 · 0.20 ms |
| 50 m | 100 | 100 · 14.33 ms | 100 · 15.81 ms | 100 · 13.28 ms | 100 · 0.44 ms | 98 · 0.43 ms |
| 100 m | 100 | 100 · 54.06 ms | 100 · 54.10 ms | 100 · 42.36 ms | 100 · 0.75 ms | 98 · 0.73 ms |
| 200 m | 100 | 100 · 286.44 ms | 58 · 135.03 ms | 100 · 221.38 ms | 100 · 3.03 ms | 73 · 2.44 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Map size | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 10 m | - | - | - | - | - |
| 25 m | - | - | - | - | 0 touching, 2 failed |
| 50 m | - | - | - | - | 0 touching, 2 failed |
| 100 m | - | - | - | - | 0 touching, 2 failed |
| 200 m | - | 0 touching, 42 failed | - | - | 0 touching, 27 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Map size | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 10 m | 1.1 · 1.000 · 4.5 | 4.7 · 0.988 · 13.8 | 3.5 · 0.983 · 10.5 | 0.7 · 0.969 · 20.6 | 0.8 · 0.969 · 20.6 |
| 25 m | 7.7 · 1.000 · 1.2 | 19.9 · 1.002 · 7.3 | 17.3 · 0.991 · 3.6 | 4.0 · 0.982 · 17.4 | 1.2 · 0.983 · 17.1 |
| 50 m | 27.5 · 1.000 · 0.4 | 72.9 · 1.004 · 6.4 | 54.8 · 0.990 · 3.2 | 2.6 · 0.983 · 16.2 | 2.6 · 0.982 · 16.2 |
| 100 m | 109.1 · 1.000 · 0.3 | 321.6 · 1.015 · 5.5 | 209.1 · 0.990 · 3.3 | 5.0 · 0.986 · 18.4 | 3.7 · 0.986 · 18.0 |
| 200 m | 518.4 · 1.000 · 0.5 | 430.1 · 1.011 · 5.7 | 824.7 · 0.991 · 2.7 | 13.3 · 0.992 · 23.9 | 4.7 · 0.987 · 21.1 |

## Obstacle shape

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle shape | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| square | 100 | 100 · 13.08 ms | 100 · 11.73 ms | 100 · 10.62 ms | 100 · 0.35 ms | 100 · 0.33 ms |
| u | 98 | 93 · 13.18 ms | 93 · 17.86 ms | 97 · 16.47 ms | 98 · 2.34 ms | 43 · 0.56 ms |
| circle | 100 | 100 · 14.33 ms | 100 · 15.81 ms | 100 · 13.28 ms | 100 · 0.44 ms | 98 · 0.43 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle shape | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| square | - | - | - | - | - |
| u | 5 touching, 0 failed | 5 touching, 0 failed | 1 touching, 0 failed | - | 0 touching, 55 failed |
| circle | - | - | - | - | 0 touching, 2 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle shape | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| square | 25.7 · 1.000 · 0.6 | 89.8 · 1.002 · 9.9 | 61.6 · 0.987 · 4.7 | 2.1 · 0.977 · 19.6 | 2.1 · 0.977 · 19.6 |
| u | 25.9 · 1.000 · 5.4 | 109.6 · 1.002 · 17.1 | 85.0 · 0.986 · 13.3 | 6.2 · 0.975 · 42.8 | 3.0 · 0.987 · 30.8 |
| circle | 27.5 · 1.000 · 0.4 | 72.9 · 1.004 · 6.4 | 54.8 · 0.990 · 3.2 | 2.6 · 0.983 · 16.2 | 2.6 · 0.982 · 16.2 |

## Corridor width

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Corridor width | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| 0.8 m | 100 | 99 · 4.21 ms | 54 · 9.46 ms | 100 · 13.15 ms | 100 · 1.58 ms | 64 · 1.23 ms |
| 1.2 m | 100 | 100 · 7.31 ms | 100 · 13.25 ms | 100 · 12.47 ms | 100 · 0.68 ms | 87 · 0.56 ms |
| 2 m | 100 | 100 · 7.44 ms | 100 · 11.22 ms | 100 · 13.78 ms | 100 · 0.53 ms | 97 · 0.49 ms |
| 3 m | 100 | 100 · 9.18 ms | 100 · 10.87 ms | 100 · 11.21 ms | 100 · 0.34 ms | 100 · 0.35 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Corridor width | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0.8 m | 1 touching, 0 failed | 46 touching, 0 failed | - | - | 0 touching, 36 failed |
| 1.2 m | - | - | - | - | 0 touching, 13 failed |
| 2 m | - | - | - | - | 0 touching, 3 failed |
| 3 m | - | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Corridor width | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0.8 m | 8.4 · 1.000 · 147.7 | 44.9 · 0.987 · 150.5 | 50.3 · 0.991 · 152.2 | 2.8 · 1.007 · 147.2 | 2.0 · 1.007 · 147.2 |
| 1.2 m | 18.1 · 1.000 · 74.3 | 76.7 · 0.963 · 84.0 | 58.4 · 0.966 · 83.3 | 2.7 · 0.975 · 82.7 | 2.1 · 0.976 · 83.1 |
| 2 m | 14.3 · 1.000 · 0.7 | 92.8 · 0.996 · 10.2 | 76.5 · 0.989 · 3.9 | 2.7 · 0.976 · 23.0 | 2.1 · 0.977 · 23.3 |
| 3 m | 17.1 · 1.000 · 0.3 | 112.0 · 1.009 · 6.8 | 86.8 · 0.991 · 2.5 | 2.2 · 0.983 · 12.7 | 2.2 · 0.983 · 12.7 |

