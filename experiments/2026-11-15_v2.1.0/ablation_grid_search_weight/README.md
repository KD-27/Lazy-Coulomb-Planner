# Ablation: weight of the fallback's A* (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

All variants keep the grid fallback and solve every routable pair (704 of 704), so the differences are in time and path shape.

## Findings

| Weight | Mean of median | 95th pct | Slowest | Length vs NavFn |
|---|---|---|---|---|
| 1 (shortest grid route) | 1.97 ms | 12.1 ms | 37.7 ms | 0.968 |
| 2 | 1.27 ms | 3.9 ms | 10.3 ms | 0.972 |
| 3 (default) | 1.27 ms | 3.6 ms | 11.9 ms | 0.972 |
| 4 | 1.22 ms | 3.4 ms | 7.8 ms | 0.973 |
| 6 | 1.25 ms | 3.4 ms | 9.8 ms | 0.973 |

- An unweighted A* (weight 1) explores far more cells: three times the 95th percentile and slowest plan of weight 2 to 6. Any weight from 2 up removes that, at a cost of about 0.5 % in path length; beyond 2 the choice hardly matters.

## Limit

One map per configuration and a single seed for the start/goal pairs.
