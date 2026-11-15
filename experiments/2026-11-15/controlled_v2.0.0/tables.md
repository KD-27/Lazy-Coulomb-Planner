# Controlled benchmark, planner v2.0.0

## Obstacle density

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle density | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| 0 % | 100 | 100 · 11.77 ms | 100 · 9.61 ms | 100 · 11.67 ms | 100 · 0.18 ms | 100 · 0.18 ms |
| 2.5 % | 100 | 100 · 14.36 ms | 100 · 13.76 ms | 100 · 10.78 ms | 100 · 0.55 ms | 100 · 0.54 ms |
| 5 % | 100 | 100 · 16.38 ms | 100 · 14.60 ms | 99 · 12.83 ms | 100 · 0.85 ms | 100 · 0.87 ms |
| 10 % | 100 | 100 · 14.25 ms | 100 · 16.68 ms | 100 · 15.16 ms | 100 · 1.41 ms | 100 · 1.39 ms |
| 15 % | 100 | 99 · 15.97 ms | 99 · 18.64 ms | 99 · 14.94 ms | 100 · 1.83 ms | 97 · 1.89 ms |
| 20 % | 100 | 94 · 13.84 ms | 95 · 15.97 ms | 95 · 13.78 ms | 100 · 2.42 ms | 92 · 2.27 ms |
| 25 % | 100 | 81 · 13.31 ms | 80 · 14.72 ms | 83 · 13.77 ms | 100 · 7.58 ms | 72 · 3.59 ms |
| 30 % | 94 | 84 · 13.75 ms | 73 · 16.90 ms | 70 · 14.94 ms | 94 · 15.96 ms | 52 · 3.41 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle density | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0 % | - | - | - | - | - |
| 2.5 % | - | - | - | - | - |
| 5 % | - | - | 1 touching, 0 failed | - | - |
| 10 % | - | - | - | - | - |
| 15 % | 1 touching, 0 failed | 1 touching, 0 failed | 1 touching, 0 failed | - | 0 touching, 3 failed |
| 20 % | 6 touching, 0 failed | 5 touching, 0 failed | 5 touching, 0 failed | - | 0 touching, 8 failed |
| 25 % | 19 touching, 0 failed | 20 touching, 0 failed | 17 touching, 0 failed | - | 0 touching, 28 failed |
| 30 % | 10 touching, 0 failed | 21 touching, 0 failed | 24 touching, 0 failed | - | 0 touching, 42 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle density | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0 % | 19.6 · 1.000 · 0.0 | 91.7 · 1.014 · 0.0 | 66.3 · 1.000 · 0.0 | 0.4 · 0.997 · 0.0 | 0.4 · 0.997 · 0.0 |
| 2.5 % | 25.8 · 1.000 · 0.0 | 90.4 · 1.017 · 0.8 | 47.7 · 0.999 · 0.2 | 13.3 · 0.994 · 1.7 | 13.3 · 0.994 · 1.7 |
| 5 % | 28.0 · 1.000 · 0.0 | 84.7 · 1.013 · 2.8 | 60.0 · 0.997 · 1.0 | 16.9 · 0.992 · 6.5 | 14.5 · 0.992 · 6.5 |
| 10 % | 27.9 · 1.000 · 0.4 | 81.0 · 1.004 · 6.4 | 59.4 · 0.990 · 3.2 | 11.3 · 0.982 · 20.2 | 10.7 · 0.982 · 20.2 |
| 15 % | 25.6 · 1.000 · 2.1 | 82.9 · 0.994 · 12.3 | 52.7 · 0.985 · 7.3 | 57.7 · 0.973 · 33.2 | 19.1 · 0.973 · 33.2 |
| 20 % | 26.1 · 1.000 · 7.7 | 117.7 · 0.977 · 21.5 | 87.6 · 0.970 · 15.1 | 60.8 · 0.968 · 48.5 | 21.9 · 0.973 · 48.7 |
| 25 % | 27.1 · 1.000 · 15.5 | 179.9 · 0.958 · 29.7 | 125.0 · 0.962 · 24.9 | 63.4 · 0.953 · 60.7 | 22.6 · 0.965 · 59.0 |
| 30 % | 29.6 · 1.000 · 29.6 | 152.3 · 0.965 · 42.4 | 107.9 · 0.978 · 37.2 | 61.9 · 0.958 · 74.8 | 25.4 · 1.002 · 74.8 |

## Map size

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Map size | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| 10 m | 100 | 100 · 0.77 ms | 100 · 1.46 ms | 100 · 0.94 ms | 100 · 0.26 ms | 100 · 0.27 ms |
| 25 m | 100 | 100 · 4.12 ms | 100 · 4.80 ms | 100 · 3.85 ms | 100 · 0.66 ms | 100 · 0.67 ms |
| 50 m | 100 | 100 · 14.25 ms | 100 · 16.68 ms | 100 · 15.16 ms | 100 · 1.41 ms | 100 · 1.39 ms |
| 100 m | 100 | 100 · 53.97 ms | 100 · 54.71 ms | 100 · 42.65 ms | 100 · 2.38 ms | 99 · 2.33 ms |
| 200 m | 100 | 100 · 287.83 ms | 58 · 135.32 ms | 100 · 220.90 ms | 100 · 4.46 ms | 97 · 4.33 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Map size | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 10 m | - | - | - | - | - |
| 25 m | - | - | - | - | - |
| 50 m | - | - | - | - | - |
| 100 m | - | - | - | - | 0 touching, 1 failed |
| 200 m | - | 0 touching, 42 failed | - | - | 0 touching, 3 failed |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Map size | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 10 m | 1.1 · 1.000 · 4.5 | 4.7 · 0.988 · 13.8 | 4.1 · 0.983 · 10.5 | 5.0 · 0.969 · 21.9 | 7.3 · 0.969 · 21.9 |
| 25 m | 7.3 · 1.000 · 1.2 | 22.8 · 1.002 · 7.3 | 16.6 · 0.991 · 3.6 | 10.7 · 0.983 · 19.3 | 12.1 · 0.983 · 19.3 |
| 50 m | 27.9 · 1.000 · 0.4 | 81.0 · 1.004 · 6.4 | 59.4 · 0.990 · 3.2 | 11.3 · 0.982 · 20.2 | 10.7 · 0.982 · 20.2 |
| 100 m | 109.6 · 1.000 · 0.3 | 323.8 · 1.015 · 5.5 | 209.3 · 0.990 · 3.3 | 93.8 · 0.986 · 19.1 | 12.0 · 0.986 · 19.1 |
| 200 m | 517.1 · 1.000 · 0.5 | 434.1 · 1.011 · 5.7 | 821.5 · 0.991 · 2.7 | 531.4 · 0.993 · 22.0 | 14.7 · 0.993 · 22.0 |

## Obstacle shape

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle shape | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| square | 100 | 100 · 13.47 ms | 100 · 11.46 ms | 100 · 10.77 ms | 100 · 1.16 ms | 100 · 1.14 ms |
| u | 98 | 93 · 13.39 ms | 93 · 17.58 ms | 97 · 15.81 ms | 98 · 12.51 ms | 53 · 2.06 ms |
| circle | 100 | 100 · 14.25 ms | 100 · 16.68 ms | 100 · 15.16 ms | 100 · 1.41 ms | 100 · 1.39 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle shape | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| square | - | - | - | - | - |
| u | 5 touching, 0 failed | 5 touching, 0 failed | 1 touching, 0 failed | - | 0 touching, 45 failed |
| circle | - | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle shape | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| square | 25.8 · 1.000 · 0.6 | 89.1 · 1.002 · 9.9 | 58.7 · 0.987 · 4.7 | 3.6 · 0.976 · 21.6 | 3.6 · 0.976 · 21.6 |
| u | 26.0 · 1.000 · 5.4 | 110.9 · 1.002 · 17.1 | 85.5 · 0.986 · 13.3 | 69.6 · 0.977 · 39.1 | 16.9 · 0.993 · 32.7 |
| circle | 27.9 · 1.000 · 0.4 | 81.0 · 1.004 · 6.4 | 59.4 · 0.990 · 3.2 | 11.3 · 0.982 · 20.2 | 10.7 · 0.982 · 20.2 |

## Corridor width

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Corridor width | Routable | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|---|
| 0.8 m | 100 | 99 · 4.14 ms | 54 · 9.27 ms | 100 · 13.23 ms | 100 · 2.82 ms | 100 · 2.79 ms |
| 1.2 m | 100 | 100 · 7.30 ms | 100 · 13.23 ms | 100 · 12.44 ms | 100 · 1.99 ms | 99 · 1.93 ms |
| 2 m | 100 | 100 · 7.45 ms | 100 · 11.28 ms | 100 · 13.69 ms | 100 · 1.64 ms | 100 · 1.63 ms |
| 3 m | 100 | 100 · 9.02 ms | 100 · 10.42 ms | 100 · 11.40 ms | 100 · 1.20 ms | 100 · 1.20 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Corridor width | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0.8 m | 1 touching, 0 failed | 46 touching, 0 failed | - | - | - |
| 1.2 m | - | - | - | - | 0 touching, 1 failed |
| 2 m | - | - | - | - | - |
| 3 m | - | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Corridor width | Navfn | Smac2d | ThetaStar | LCP | LCP_lazy |
|---|---|---|---|---|---|
| 0.8 m | 8.6 · 1.000 · 147.7 | 44.4 · 0.987 · 150.5 | 50.2 · 0.991 · 152.2 | 9.5 · 1.008 · 147.1 | 8.7 · 1.008 · 147.1 |
| 1.2 m | 18.2 · 1.000 · 74.3 | 76.5 · 0.963 · 84.0 | 60.1 · 0.966 · 83.3 | 8.4 · 0.977 · 82.7 | 9.0 · 0.977 · 82.7 |
| 2 m | 14.0 · 1.000 · 0.7 | 92.7 · 0.996 · 10.2 | 79.6 · 0.989 · 3.9 | 7.3 · 0.976 · 23.8 | 6.9 · 0.976 · 23.8 |
| 3 m | 16.9 · 1.000 · 0.3 | 111.9 · 1.009 · 6.8 | 90.3 · 0.991 · 2.5 | 3.1 · 0.983 · 12.4 | 4.1 · 0.983 · 12.4 |

