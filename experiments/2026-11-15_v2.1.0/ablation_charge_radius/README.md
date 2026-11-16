# Ablation: charge radius (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

All variants keep the grid fallback and solve every routable pair (704 of 704), so the differences are in time and path shape.

## Findings

| Radius | Mean of median time | 95th pct | Slowest | Mean cost along the path |
|---|---|---|---|---|
| 0.5 m | 0.88 ms | 2.6 ms | 6.9 ms | 41.8 |
| 1.0 m (default) | 1.27 ms | 3.6 ms | 12.5 ms | 39.2 |
| 2.0 m | 2.29 ms | 6.2 ms | 17.7 ms | 38.9 |
| 4.0 m | 5.76 ms | 13.9 ms | 30.7 ms | 38.4 |

- Time grows steeply with the radius (the force sum visits a number of cells that grows with its square); clearance improves by only a few percent.
- 0.5 m is 30 % faster than the default, with paths slightly closer to obstacles.

## Limit

One map per configuration and a single seed for the start/goal pairs.
