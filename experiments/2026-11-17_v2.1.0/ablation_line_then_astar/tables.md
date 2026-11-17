# Ablation: straight line, else A*, planner v2.1.0

## Obstacle density

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle density | Routable | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|---|
| 0 % | 100 | 100 · 11.02 ms | 100 · 9.25 ms | 100 · 0.07 ms | 100 · 0.07 ms | 100 · 0.06 ms | 100 · 0.07 ms |
| 2.5 % | 100 | 100 · 15.03 ms | 100 · 13.83 ms | 100 · 0.16 ms | 100 · 0.42 ms | 100 · 3.62 ms | 100 · 0.13 ms |
| 5 % | 100 | 100 · 15.50 ms | 100 · 13.63 ms | 100 · 0.25 ms | 100 · 0.48 ms | 100 · 4.51 ms | 100 · 0.27 ms |
| 10 % | 100 | 100 · 14.10 ms | 100 · 15.85 ms | 100 · 0.43 ms | 100 · 0.67 ms | 100 · 5.25 ms | 100 · 0.71 ms |
| 15 % | 100 | 99 · 15.63 ms | 99 · 18.61 ms | 100 · 0.63 ms | 100 · 0.80 ms | 100 · 4.70 ms | 100 · 1.05 ms |
| 20 % | 100 | 94 · 13.62 ms | 95 · 15.08 ms | 100 · 1.17 ms | 100 · 0.90 ms | 100 · 4.70 ms | 99 · 2.45 ms |
| 25 % | 100 | 81 · 12.61 ms | 80 · 14.17 ms | 100 · 2.91 ms | 100 · 1.11 ms | 100 · 4.55 ms | 100 · 3.33 ms |
| 30 % | 94 | 84 · 13.58 ms | 73 · 17.05 ms | 94 · 3.83 ms | 94 · 1.47 ms | 94 · 5.45 ms | 93 · 3.88 ms |
| 35 % | 71 | 57 · 16.18 ms | 42 · 23.42 ms | 71 · 4.16 ms | 71 · 1.89 ms | 71 · 5.87 ms | 71 · 4.50 ms |
| 40 % | 74 | 53 · 18.47 ms | 57 · 18.59 ms | 74 · 5.56 ms | 74 · 2.98 ms | 74 · 5.79 ms | 74 · 5.72 ms |
| 45 % | 30 | 15 · 14.75 ms | 13 · 10.96 ms | 30 · 5.27 ms | 30 · 2.82 ms | 30 · 3.85 ms | 30 · 5.35 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle density | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| 0 % | - | - | - | - | - | - |
| 2.5 % | - | - | - | - | - | - |
| 5 % | - | - | - | - | - | - |
| 10 % | - | - | - | - | - | - |
| 15 % | 1 touching, 0 failed | 1 touching, 0 failed | - | - | - | - |
| 20 % | 6 touching, 0 failed | 5 touching, 0 failed | - | - | - | 1 touching, 0 failed |
| 25 % | 19 touching, 0 failed | 20 touching, 0 failed | - | - | - | - |
| 30 % | 10 touching, 0 failed | 21 touching, 0 failed | - | - | - | 1 touching, 0 failed |
| 35 % | 14 touching, 0 failed | 29 touching, 0 failed | - | - | - | - |
| 40 % | 21 touching, 0 failed | 17 touching, 0 failed | - | - | - | - |
| 45 % | 15 touching, 0 failed | 17 touching, 0 failed | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle density | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| 0 % | 20.3 · 1.000 · 0.0 | 86.8 · 1.014 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 |
| 2.5 % | 24.6 · 1.000 · 0.0 | 88.6 · 1.017 · 0.8 | 2.1 · 0.994 · 1.9 | 0.8 · 0.996 · 1.9 | 34.5 · 0.996 · 1.8 | 0.7 · 0.996 · 6.8 |
| 5 % | 26.3 · 1.000 · 0.0 | 79.9 · 1.013 · 2.8 | 2.5 · 0.992 · 6.3 | 1.2 · 0.995 · 7.0 | 35.4 · 0.993 · 7.4 | 2.4 · 0.995 · 19.7 |
| 10 % | 26.9 · 1.000 · 0.4 | 74.7 · 1.004 · 6.4 | 3.6 · 0.983 · 16.2 | 1.3 · 0.988 · 25.1 | 29.2 · 0.983 · 25.1 | 4.1 · 0.997 · 41.9 |
| 15 % | 25.2 · 1.000 · 2.1 | 79.6 · 0.994 · 12.3 | 4.8 · 0.975 · 32.6 | 1.5 · 0.976 · 36.0 | 27.2 · 0.954 · 38.7 | 4.6 · 0.984 · 56.7 |
| 20 % | 25.3 · 1.000 · 7.7 | 110.6 · 0.977 · 21.5 | 5.6 · 0.959 · 48.6 | 4.0 · 0.954 · 53.8 | 35.1 · 0.938 · 51.1 | 5.6 · 0.965 · 79.7 |
| 25 % | 26.6 · 1.000 · 15.5 | 164.5 · 0.958 · 29.7 | 10.4 · 0.936 · 60.4 | 8.6 · 0.940 · 61.5 | 49.5 · 0.920 · 58.2 | 11.1 · 0.902 · 102.8 |
| 30 % | 28.9 · 1.000 · 29.6 | 151.5 · 0.965 · 42.4 | 8.2 · 0.951 · 72.8 | 4.8 · 0.954 · 74.0 | 21.5 · 0.930 · 74.2 | 8.8 · 0.914 · 125.1 |
| 35 % | 27.6 · 1.000 · 43.5 | 105.8 · 0.948 · 60.5 | 10.9 · 0.928 · 91.7 | 9.1 · 0.926 · 93.0 | 26.9 · 0.904 · 92.6 | 11.2 · 0.858 · 141.7 |
| 40 % | 95.2 · 1.000 · 68.1 | 87.6 · 0.962 · 80.5 | 18.0 · 0.956 · 101.5 | 11.8 · 0.958 · 101.6 | 23.1 · 0.939 · 104.2 | 15.1 · 0.894 · 158.1 |
| 45 % | 68.6 · 1.000 · 86.5 | 38.2 · 0.963 · 86.8 | 14.8 · 0.938 · 113.8 | 10.5 · 0.935 · 115.6 | 11.8 · 0.936 · 117.9 | 12.9 · 0.880 · 169.7 |

## Map size

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Map size | Routable | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|---|
| 10 m | 100 | 100 · 0.76 ms | 100 · 1.42 ms | 100 · 0.10 ms | 100 · 0.13 ms | 100 · 0.29 ms | 100 · 0.08 ms |
| 25 m | 100 | 100 · 4.06 ms | 100 · 4.95 ms | 100 · 0.21 ms | 100 · 0.29 ms | 100 · 1.69 ms | 100 · 0.29 ms |
| 50 m | 100 | 100 · 14.10 ms | 100 · 15.85 ms | 100 · 0.43 ms | 100 · 0.67 ms | 100 · 5.25 ms | 100 · 0.71 ms |
| 100 m | 100 | 100 · 52.19 ms | 100 · 54.45 ms | 100 · 0.73 ms | 100 · 2.01 ms | 100 · 20.23 ms | 100 · 1.40 ms |
| 200 m | 100 | 100 · 303.78 ms | 58 · 157.29 ms | 100 · 3.29 ms | 100 · 4.64 ms | 100 · 102.14 ms | 100 · 5.68 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Map size | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| 10 m | - | - | - | - | - | - |
| 25 m | - | - | - | - | - | - |
| 50 m | - | - | - | - | - | - |
| 100 m | - | - | - | - | - | - |
| 200 m | - | 0 touching, 42 failed | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Map size | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| 10 m | 1.1 · 1.000 · 4.5 | 4.8 · 0.988 · 13.8 | 0.7 · 0.969 · 20.6 | 0.4 · 0.972 · 18.6 | 1.5 · 0.972 · 19.5 | 3.6 · 0.960 · 44.8 |
| 25 m | 7.1 · 1.000 · 1.2 | 20.0 · 1.002 · 7.3 | 3.1 · 0.982 · 18.1 | 0.8 · 0.984 · 21.2 | 7.9 · 0.980 · 19.5 | 2.8 · 0.990 · 35.6 |
| 50 m | 26.9 · 1.000 · 0.4 | 74.7 · 1.004 · 6.4 | 3.6 · 0.983 · 16.2 | 1.3 · 0.988 · 25.1 | 29.2 · 0.983 · 25.1 | 4.1 · 0.997 · 41.9 |
| 100 m | 107.9 · 1.000 · 0.3 | 323.5 · 1.015 · 5.5 | 4.6 · 0.986 · 18.5 | 3.8 · 0.998 · 27.2 | 114.1 · 0.985 · 25.6 | 6.4 · 1.002 · 37.4 |
| 200 m | 727.8 · 1.000 · 0.5 | 522.5 · 1.011 · 5.7 | 16.8 · 0.992 · 23.9 | 12.2 · 1.005 · 28.8 | 474.5 · 0.988 · 29.0 | 15.7 · 0.999 · 43.3 |

## Obstacle shape

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle shape | Routable | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|---|
| square | 100 | 100 · 14.45 ms | 100 · 13.40 ms | 100 · 0.40 ms | 100 · 0.71 ms | 100 · 5.10 ms | 100 · 0.66 ms |
| u | 98 | 93 · 14.69 ms | 93 · 18.79 ms | 98 · 2.83 ms | 98 · 1.14 ms | 98 · 6.54 ms | 98 · 3.54 ms |
| circle | 100 | 100 · 14.10 ms | 100 · 15.85 ms | 100 · 0.43 ms | 100 · 0.67 ms | 100 · 5.25 ms | 100 · 0.71 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle shape | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| square | - | - | - | - | - | - |
| u | 5 touching, 0 failed | 5 touching, 0 failed | - | - | - | - |
| circle | - | - | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle shape | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| square | 34.3 · 1.000 · 0.6 | 158.6 · 1.001 · 9.9 | 2.5 · 0.977 · 19.4 | 1.6 · 0.973 · 28.1 | 32.3 · 0.967 · 26.3 | 3.6 · 1.008 · 35.7 |
| u | 38.4 · 1.000 · 5.4 | 152.3 · 1.002 · 17.1 | 7.7 · 0.974 · 42.8 | 4.2 · 0.975 · 42.4 | 36.9 · 0.966 · 39.5 | 9.3 · 0.960 · 74.1 |
| circle | 26.9 · 1.000 · 0.4 | 74.7 · 1.004 · 6.4 | 3.6 · 0.983 · 16.2 | 1.3 · 0.988 · 25.1 | 29.2 · 0.983 · 25.1 | 4.1 · 0.997 · 41.9 |

## Corridor width

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Corridor width | Routable | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|---|
| 0.8 m | 100 | 99 · 4.75 ms | 54 · 10.27 ms | 100 · 1.63 ms | 100 · 0.83 ms | 100 · 2.69 ms | 100 · 1.65 ms |
| 1.2 m | 100 | 100 · 8.03 ms | 100 · 14.43 ms | 100 · 0.69 ms | 100 · 0.85 ms | 100 · 3.56 ms | 100 · 1.24 ms |
| 2 m | 100 | 100 · 8.00 ms | 100 · 13.83 ms | 100 · 0.57 ms | 100 · 0.86 ms | 100 · 5.62 ms | 100 · 1.25 ms |
| 3 m | 100 | 100 · 8.94 ms | 100 · 11.57 ms | 100 · 0.35 ms | 100 · 0.59 ms | 100 · 4.88 ms | 100 · 0.87 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Corridor width | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| 0.8 m | 1 touching, 0 failed | 46 touching, 0 failed | - | - | - | - |
| 1.2 m | - | - | - | - | - | - |
| 2 m | - | - | - | - | - | - |
| 3 m | - | - | - | - | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Corridor width | Navfn | Smac2d | LCP | line_astar_w3 | line_astar_w1 | fixed_left |
|---|---|---|---|---|---|---|
| 0.8 m | 9.0 · 1.000 · 147.7 | 44.3 · 0.987 · 150.5 | 4.6 · 1.007 · 147.2 | 2.1 · 1.008 · 147.1 | 16.3 · 1.011 · 146.7 | 5.3 · 1.029 · 171.3 |
| 1.2 m | 18.9 · 1.000 · 74.3 | 83.4 · 0.964 · 84.0 | 2.9 · 0.975 · 82.7 | 1.9 · 0.980 · 80.0 | 21.3 · 0.980 · 79.7 | 5.0 · 1.030 · 126.6 |
| 2 m | 15.6 · 1.000 · 0.7 | 96.6 · 0.996 · 10.2 | 3.0 · 0.976 · 23.3 | 1.6 · 0.977 · 16.3 | 34.2 · 0.975 · 17.3 | 5.8 · 1.064 · 70.8 |
| 3 m | 18.7 · 1.000 · 0.3 | 116.9 · 1.009 · 6.8 | 2.1 · 0.983 · 12.7 | 2.3 · 0.976 · 21.0 | 40.7 · 0.966 · 25.3 | 4.2 · 1.082 · 46.4 |

