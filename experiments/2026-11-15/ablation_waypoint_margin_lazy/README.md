# Ablation: waypoint margin, lazy search alone (planner v2.0.0)

The same four margins as `../ablation_waypoint_margin/` (0, 0.15, 0.3 = default,
0.5 m) with the grid fallback switched **off**, so the numbers show what each
margin lets the lazy search solve by itself. Eight maps, 100 pairs each, every
pair planned 3 times. Full tables: `tables.md`.

## Findings

| Margin | Routable pairs solved (of 704) | Maze (100) | 30 % density (94) | U shapes (98) |
|---|---|---|---|---|
| 0 m | 505 (72 %) | 68 | 31 | 43 |
| 0.15 m | 560 (80 %) | 79 | 45 | 52 |
| 0.3 m (default) | 582 (83 %) | 82 | 51 | 53 |
| 0.5 m | 601 (85 %) | 90 | 56 | 58 |

- The margin is one of the strongest levers on the lazy search: with none it
  solves 72 % of the routable pairs, with 0.5 m 85 %. A waypoint pushed a little
  past the obstacle's edge clears the corner, so the next segment is less likely
  to hit the same obstacle again.
- The gain continues beyond the default. 0.5 m solves 19 more pairs than 0.3 m,
  8 of them on the maze, at no cost in planning time (median on the maze 2.0 ms
  against 1.8 ms) and none in path length (see `../ablation_waypoint_margin/`).
- Together with the result there (0.5 m is also the fastest with the fallback
  on), this is a clear case for raising the default from 0.3 to 0.5 m in the
  next planner version, and for testing larger values.

## Limit

One map per configuration and a single seed for the start/goal pairs.
