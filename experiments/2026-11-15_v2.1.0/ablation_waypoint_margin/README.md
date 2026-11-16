# Ablation: waypoint margin (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

All variants keep the grid fallback and solve every routable pair (704 of 704), so the differences are in time and path shape.

## Findings

| Margin | Mean of median time | Slowest | Length vs NavFn | Mean cost along the path |
|---|---|---|---|---|
| 0 m | 1.58 ms | 8.3 ms | 0.972 | 39.0 |
| 0.25 m | 1.39 ms | 9.1 ms | 0.971 | 39.6 |
| 0.5 m (default) | 1.24 ms | 10.2 ms | 0.972 | 39.2 |
| 0.75 m | 1.24 ms | 9.1 ms | 0.975 | 37.7 |
| 1.0 m | 1.16 ms | 11.0 ms | 0.979 | 35.2 |

- A larger margin makes planning a little faster and paths a little further from obstacles (cost 39 to 35), at the price of length (0.7 % longer at 1 m than at 0.5 m).

## Limit

One map per configuration and a single seed for the start/goal pairs.
