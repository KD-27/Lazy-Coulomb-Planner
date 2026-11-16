# Ablation: waypoint margin, lazy search alone (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

The grid fallback is **off** in every variant, so the numbers show what the lazy search solves by itself.

## Findings

| Margin | Solved by the lazy search (of 704) | 30 % density | U shapes | Maze |
|---|---|---|---|---|
| 0 m | 305 (43 %) | 11 | 23 | 37 |
| 0.25 m | 470 (67 %) | 31 | 38 | 61 |
| 0.5 m (default) | 489 (69 %) | 35 | 43 | 58 |
| 0.75 m | 499 (71 %) | 31 | 39 | 59 |
| 1.0 m | 509 (72 %) | 35 | 41 | 60 |

- The margin remains the strongest single lever on the lazy search: without it, it solves 43 % of the pairs; with 0.5 m, 69 %. Beyond 0.5 m the gain flattens (72 % at 1 m). The default of 0.5 m sits at the knee.

## Limit

One map per configuration and a single seed for the start/goal pairs.
