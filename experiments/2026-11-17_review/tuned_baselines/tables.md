# Tuned grid baselines, planner v2.1.0

## Obstacle density

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle density | Routable | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|---|
| 0 % | 100 | 100 · 10.79 ms | 100 · 3.28 ms | 100 · 10.41 ms | 100 · 9.79 ms | 100 · 11.28 ms | 100 · 8.46 ms | 100 · 0.07 ms | 100 · 0.06 ms |
| 2.5 % | 100 | 100 · 15.01 ms | 100 · 3.51 ms | 74 · 13.99 ms | 73 · 14.18 ms | 100 · 11.26 ms | 70 · 11.85 ms | 100 · 0.15 ms | 100 · 0.44 ms |
| 5 % | 100 | 100 · 16.33 ms | 100 · 4.06 ms | 63 · 12.25 ms | 62 · 12.77 ms | 99 · 12.46 ms | 49 · 13.71 ms | 100 · 0.25 ms | 100 · 0.57 ms |
| 10 % | 100 | 100 · 14.03 ms | 100 · 4.71 ms | 51 · 11.00 ms | 51 · 11.49 ms | 100 · 14.00 ms | 22 · 12.65 ms | 100 · 0.43 ms | 100 · 0.70 ms |
| 15 % | 100 | 99 · 15.97 ms | 99 · 5.79 ms | 46 · 9.51 ms | 45 · 9.38 ms | 99 · 16.12 ms | 11 · 11.97 ms | 100 · 0.65 ms | 100 · 0.82 ms |
| 20 % | 100 | 94 · 15.54 ms | 93 · 6.09 ms | 31 · 9.04 ms | 30 · 8.98 ms | 95 · 14.04 ms | 7 · 11.24 ms | 100 · 1.19 ms | 100 · 0.95 ms |
| 25 % | 100 | 81 · 12.82 ms | 81 · 5.89 ms | 30 · 8.23 ms | 29 · 8.31 ms | 83 · 12.72 ms | 5 · 12.71 ms | 100 · 2.89 ms | 100 · 1.13 ms |
| 30 % | 94 | 84 · 13.93 ms | 83 · 5.85 ms | 33 · 9.96 ms | 32 · 9.93 ms | 70 · 15.49 ms | 4 · 14.21 ms | 94 · 3.71 ms | 94 · 1.43 ms |
| 35 % | 71 | 57 · 16.66 ms | 50 · 6.06 ms | 17 · 10.60 ms | 16 · 10.60 ms | 38 · 17.92 ms | 0 · 15.64 ms | 71 · 4.01 ms | 71 · 1.74 ms |
| 40 % | 74 | 52 · 21.74 ms | 39 · 6.08 ms | 10 · 9.61 ms | 10 · 9.88 ms | 38 · 16.73 ms | 0 · 14.51 ms | 74 · 5.99 ms | 74 · 3.06 ms |
| 45 % | 30 | 15 · 16.98 ms | 13 · 5.76 ms | 1 · 7.46 ms | 1 · 7.65 ms | 10 · 12.19 ms | 0 · 9.74 ms | 30 · 6.67 ms | 30 · 3.49 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle density | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| 0 % | - | - | - | - | - | - | - | - |
| 2.5 % | - | - | 26 touching, 0 failed | 27 touching, 0 failed | - | 30 touching, 0 failed | - | - |
| 5 % | - | - | 37 touching, 0 failed | 38 touching, 0 failed | 1 touching, 0 failed | 51 touching, 0 failed | - | - |
| 10 % | - | - | 49 touching, 0 failed | 49 touching, 0 failed | - | 78 touching, 0 failed | - | - |
| 15 % | 1 touching, 0 failed | 1 touching, 0 failed | 54 touching, 0 failed | 55 touching, 0 failed | 1 touching, 0 failed | 89 touching, 0 failed | - | - |
| 20 % | 6 touching, 0 failed | 6 touching, 1 failed | 69 touching, 0 failed | 70 touching, 0 failed | 5 touching, 0 failed | 93 touching, 0 failed | - | - |
| 25 % | 19 touching, 0 failed | 19 touching, 0 failed | 70 touching, 0 failed | 71 touching, 0 failed | 17 touching, 0 failed | 95 touching, 0 failed | - | - |
| 30 % | 10 touching, 0 failed | 11 touching, 0 failed | 61 touching, 0 failed | 62 touching, 0 failed | 24 touching, 0 failed | 90 touching, 0 failed | - | - |
| 35 % | 14 touching, 0 failed | 21 touching, 0 failed | 54 touching, 0 failed | 55 touching, 0 failed | 33 touching, 0 failed | 71 touching, 0 failed | - | - |
| 40 % | 22 touching, 0 failed | 35 touching, 0 failed | 64 touching, 0 failed | 64 touching, 0 failed | 36 touching, 0 failed | 74 touching, 0 failed | - | - |
| 45 % | 15 touching, 0 failed | 17 touching, 0 failed | 29 touching, 0 failed | 29 touching, 0 failed | 20 touching, 0 failed | 30 touching, 0 failed | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle density | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| 0 % | 19.8 · 1.000 · 0.0 | 5.2 · 1.000 · 0.0 | 89.0 · 1.014 · 0.0 | 88.1 · 1.014 · 0.0 | 63.8 · 1.000 · 0.0 | 112.4 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 | 0.2 · 0.997 · 0.0 |
| 2.5 % | 25.6 · 1.000 · 0.0 | 8.9 · 1.000 · 0.0 | 88.5 · 1.014 · 9.3 | 88.2 · 1.014 · 9.5 | 48.3 · 0.999 · 0.2 | 191.7 · 0.992 · 8.6 | 3.6 · 0.994 · 1.9 | 0.8 · 0.996 · 1.9 |
| 5 % | 30.1 · 1.000 · 0.0 | 10.4 · 1.004 · 0.1 | 88.8 · 1.011 · 28.8 | 88.4 · 1.012 · 28.9 | 58.5 · 0.997 · 1.0 | 238.4 · 0.989 · 17.6 | 4.3 · 0.992 · 6.3 | 1.4 · 0.995 · 7.0 |
| 10 % | 28.3 · 1.000 · 0.4 | 13.9 · 1.009 · 1.0 | 68.7 · 0.989 · 53.8 | 68.9 · 0.989 · 55.2 | 56.8 · 0.990 · 3.2 | 81.1 · 0.977 · 41.6 | 3.9 · 0.983 · 16.2 | 1.4 · 0.988 · 25.1 |
| 15 % | 26.5 · 1.000 · 2.1 | 13.8 · 1.012 · 2.5 | 60.1 · 0.985 · 72.0 | 60.9 · 0.986 · 72.0 | 51.5 · 0.985 · 7.3 | 72.8 · 0.951 · 56.9 | 4.0 · 0.975 · 32.6 | 1.7 · 0.976 · 36.1 |
| 20 % | 33.2 · 1.000 · 7.7 | 24.7 · 1.011 · 8.2 | 80.6 · 0.939 · 98.8 | 80.8 · 0.939 · 99.8 | 84.5 · 0.970 · 15.1 | 153.0 · 0.973 · 57.8 | 7.4 · 0.959 · 48.6 | 5.4 · 0.954 · 53.8 |
| 25 % | 27.7 · 1.000 · 15.5 | 20.8 · 1.016 · 15.8 | 111.8 · 0.909 · 113.4 | 114.2 · 0.907 · 113.9 | 118.2 · 0.962 · 24.9 | 177.3 · 0.938 · 87.8 | 9.9 · 0.936 · 60.4 | 8.8 · 0.940 · 61.5 |
| 30 % | 28.6 · 1.000 · 29.6 | 16.4 · 1.012 · 29.9 | 112.7 · 0.924 · 117.5 | 114.8 · 0.930 · 118.2 | 107.3 · 0.978 · 37.2 | 153.0 · 0.859 · 91.8 | 8.8 · 0.951 · 72.8 | 5.3 · 0.954 · 74.0 |
| 35 % | 27.9 · 1.000 · 43.5 | 13.5 · 1.011 · 43.0 | 75.3 · 0.890 · 140.1 | 73.6 · 0.890 · 140.1 | 82.8 · 0.962 · 56.7 | 90.3 · - · - | 11.3 · 0.928 · 91.7 | 9.0 · 0.926 · 92.3 |
| 40 % | 100.9 · 1.000 · 68.2 | 18.6 · 1.007 · 65.5 | 51.7 · 0.906 · 159.9 | 61.1 · 0.906 · 159.9 | 66.8 · 0.975 · 66.0 | 66.4 · - · - | 18.6 · 0.956 · 100.0 | 15.5 · 0.957 · 100.9 |
| 45 % | 76.6 · 1.000 · 86.5 | 11.6 · 1.005 · 78.5 | 29.5 · 0.925 · 163.2 | 31.2 · 0.925 · 163.2 | 33.9 · 0.980 · 80.0 | 33.6 · - · - | 22.2 · 0.938 · 113.8 | 12.1 · 0.936 · 115.6 |

## Map size

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Map size | Routable | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|---|
| 10 m | 100 | 100 · 0.77 ms | 100 · 0.34 ms | 77 · 0.97 ms | 77 · 0.93 ms | 100 · 0.95 ms | 73 · 0.50 ms | 100 · 0.10 ms | 100 · 0.13 ms |
| 25 m | 100 | 100 · 4.08 ms | 100 · 1.31 ms | 58 · 3.65 ms | 57 · 3.97 ms | 100 · 3.96 ms | 41 · 3.08 ms | 100 · 0.21 ms | 100 · 0.28 ms |
| 50 m | 100 | 100 · 14.03 ms | 100 · 4.71 ms | 51 · 11.00 ms | 51 · 11.49 ms | 100 · 14.00 ms | 22 · 12.65 ms | 100 · 0.43 ms | 100 · 0.70 ms |
| 100 m | 100 | 100 · 54.31 ms | 100 · 16.61 ms | 60 · 48.35 ms | 60 · 48.53 ms | 100 · 46.86 ms | 13 · 49.93 ms | 100 · 0.84 ms | 100 · 2.20 ms |
| 200 m | 100 | 100 · 330.19 ms | 100 · 103.45 ms | 28 · 111.87 ms | 28 · 116.06 ms | 100 · 234.30 ms | 5 · 206.44 ms | 100 · 3.05 ms | 100 · 4.87 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Map size | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| 10 m | - | - | 23 touching, 0 failed | 23 touching, 0 failed | - | 27 touching, 0 failed | - | - |
| 25 m | - | - | 42 touching, 0 failed | 43 touching, 0 failed | - | 59 touching, 0 failed | - | - |
| 50 m | - | - | 49 touching, 0 failed | 49 touching, 0 failed | - | 78 touching, 0 failed | - | - |
| 100 m | - | - | 40 touching, 0 failed | 40 touching, 0 failed | - | 87 touching, 0 failed | - | - |
| 200 m | - | - | 36 touching, 36 failed | 36 touching, 36 failed | - | 95 touching, 0 failed | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Map size | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| 10 m | 1.1 · 1.000 · 4.5 | 1.4 · 1.004 · 5.0 | 4.1 · 0.977 · 45.9 | 3.0 · 0.977 · 47.5 | 4.0 · 0.983 · 10.5 | 3.5 · 0.955 · 46.2 | 0.8 · 0.969 · 20.6 | 0.4 · 0.972 · 17.7 |
| 25 m | 7.3 · 1.000 · 1.2 | 5.9 · 1.006 · 1.5 | 22.0 · 0.999 · 45.0 | 18.2 · 0.999 · 45.1 | 17.9 · 0.991 · 3.6 | 43.7 · 0.976 · 32.8 | 4.0 · 0.982 · 17.4 | 0.8 · 0.984 · 20.9 |
| 50 m | 28.3 · 1.000 · 0.4 | 13.9 · 1.009 · 1.0 | 68.7 · 0.989 · 53.8 | 68.9 · 0.989 · 55.2 | 56.8 · 0.990 · 3.2 | 81.1 · 0.977 · 41.6 | 3.9 · 0.983 · 16.2 | 1.4 · 0.988 · 25.1 |
| 100 m | 198.7 · 1.000 · 0.3 | 42.2 · 1.010 · 0.7 | 365.1 · 1.007 · 56.9 | 384.6 · 1.007 · 56.9 | 223.2 · 0.990 · 3.3 | 320.4 · 0.983 · 24.7 | 4.3 · 0.986 · 18.4 | 5.9 · 0.998 · 27.2 |
| 200 m | 603.8 · 1.000 · 0.5 | 159.7 · 1.011 · 0.7 | 461.2 · 1.003 · 53.4 | 547.9 · 1.003 · 53.4 | 823.6 · 0.991 · 2.7 | 977.2 · 0.979 · 23.7 | 13.3 · 0.992 · 23.9 | 14.5 · 1.005 · 28.8 |

## Obstacle shape

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Obstacle shape | Routable | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|---|
| square | 100 | 100 · 13.70 ms | 100 · 4.19 ms | 45 · 7.64 ms | 44 · 7.45 ms | 100 · 10.60 ms | 20 · 9.51 ms | 100 · 0.35 ms | 100 · 0.66 ms |
| u | 98 | 93 · 13.46 ms | 93 · 5.56 ms | 31 · 12.77 ms | 30 · 12.47 ms | 97 · 15.48 ms | 15 · 22.09 ms | 98 · 2.71 ms | 98 · 0.97 ms |
| circle | 100 | 100 · 14.03 ms | 100 · 4.71 ms | 51 · 11.00 ms | 51 · 11.49 ms | 100 · 14.00 ms | 22 · 12.65 ms | 100 · 0.43 ms | 100 · 0.70 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Obstacle shape | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| square | - | - | 55 touching, 0 failed | 56 touching, 0 failed | - | 80 touching, 0 failed | - | - |
| u | 5 touching, 0 failed | 4 touching, 1 failed | 67 touching, 0 failed | 68 touching, 0 failed | 1 touching, 0 failed | 83 touching, 0 failed | - | - |
| circle | - | - | 49 touching, 0 failed | 49 touching, 0 failed | - | 78 touching, 0 failed | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Obstacle shape | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| square | 26.0 · 1.000 · 0.6 | 15.7 · 1.011 · 1.0 | 61.5 · 0.987 · 60.9 | 63.4 · 0.987 · 62.2 | 60.3 · 0.987 · 4.7 | 85.6 · 0.966 · 34.7 | 2.1 · 0.977 · 19.6 | 2.4 · 0.973 · 28.1 |
| u | 25.9 · 1.000 · 5.4 | 18.6 · 1.012 · 6.5 | 82.5 · 0.990 · 89.8 | 84.6 · 0.986 · 93.1 | 86.4 · 0.986 · 13.3 | 165.3 · 0.976 · 32.9 | 6.3 · 0.975 · 42.8 | 4.2 · 0.975 · 42.4 |
| circle | 28.3 · 1.000 · 0.4 | 13.9 · 1.009 · 1.0 | 68.7 · 0.989 · 53.8 | 68.9 · 0.989 · 55.2 | 56.8 · 0.990 · 3.2 | 81.1 · 0.977 · 41.6 | 3.9 · 0.983 · 16.2 | 1.4 · 0.988 · 25.1 |

## Corridor width

Collision-free paths out of the pairs that have a route (ground truth from the map), and the median planning time.

| Corridor width | Routable | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|---|
| 0.8 m | 100 | 99 · 4.17 ms | 99 · 3.82 ms | 46 · 4.47 ms | 46 · 4.56 ms | 100 · 13.13 ms | 8 · 10.45 ms | 100 · 1.55 ms | 100 · 0.81 ms |
| 1.2 m | 100 | 100 · 7.39 ms | 100 · 4.74 ms | 55 · 5.90 ms | 55 · 5.66 ms | 100 · 12.46 ms | 11 · 13.47 ms | 100 · 0.69 ms | 100 · 0.81 ms |
| 2 m | 100 | 100 · 7.50 ms | 100 · 4.19 ms | 54 · 9.72 ms | 53 · 9.78 ms | 100 · 13.78 ms | 12 · 22.61 ms | 100 · 0.56 ms | 100 · 0.83 ms |
| 3 m | 100 | 100 · 8.85 ms | 100 · 3.80 ms | 45 · 9.01 ms | 44 · 8.56 ms | 100 · 11.47 ms | 20 · 13.55 ms | 100 · 0.35 ms | 100 · 0.56 ms |

Routable pairs not solved: paths that touch a cell where the robot would be in contact with an obstacle, and requests that failed.

| Corridor width | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| 0.8 m | 1 touching, 0 failed | 1 touching, 0 failed | 54 touching, 0 failed | 54 touching, 0 failed | - | 92 touching, 0 failed | - | - |
| 1.2 m | - | - | 45 touching, 0 failed | 45 touching, 0 failed | - | 89 touching, 0 failed | - | - |
| 2 m | - | - | 46 touching, 0 failed | 47 touching, 0 failed | - | 88 touching, 0 failed | - | - |
| 3 m | - | - | 55 touching, 0 failed | 56 touching, 0 failed | - | 80 touching, 0 failed | - | - |

Slowest plan (ms), path length relative to Navfn, and average costmap cost along the path.

| Corridor width | Navfn | Navfn_astar | Smac2d | Smac2d_nocost | ThetaStar | ThetaStar_nocost | LCP | line_astar_w3 |
|---|---|---|---|---|---|---|---|---|
| 0.8 m | 9.0 · 1.000 · 147.7 | 5.9 · 1.001 · 147.8 | 37.5 · 0.986 · 218.9 | 37.8 · 0.986 · 221.8 | 54.8 · 0.991 · 152.2 | 91.1 · 0.993 · 162.7 | 3.4 · 1.007 · 147.2 | 2.5 · 1.008 · 147.2 |
| 1.2 m | 19.4 · 1.000 · 74.3 | 11.7 · 1.000 · 74.2 | 46.6 · 0.954 · 202.1 | 47.1 · 0.954 · 202.1 | 58.3 · 0.966 · 83.3 | 87.1 · 0.982 · 121.9 | 2.9 · 0.975 · 82.7 | 2.3 · 0.979 · 80.0 |
| 2 m | 15.2 · 1.000 · 0.7 | 13.1 · 1.004 · 0.8 | 75.9 · 0.974 · 157.4 | 78.2 · 0.974 · 158.5 | 77.3 · 0.989 · 3.9 | 140.3 · 0.989 · 39.0 | 2.6 · 0.976 · 23.0 | 1.8 · 0.977 · 16.3 |
| 3 m | 17.7 · 1.000 · 0.3 | 13.1 · 1.005 · 0.4 | 87.1 · 0.997 · 90.4 | 85.2 · 0.997 · 91.0 | 87.9 · 0.991 · 2.5 | 146.8 · 0.988 · 8.2 | 1.9 · 0.983 · 12.7 | 1.3 · 0.976 · 21.0 |

