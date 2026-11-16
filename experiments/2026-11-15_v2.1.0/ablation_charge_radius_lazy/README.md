# Ablation: charge radius, lazy search alone (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

The grid fallback is **off** in every variant, so the numbers show what the lazy search solves by itself.

## Findings

| Radius | Solved by the lazy search (of 704) | 30 % density | U shapes | Maze | Mean of median time |
|---|---|---|---|---|---|
| 0.5 m | 478 | 34 | 37 | 64 | 0.30 ms |
| 1.0 m (default) | 489 | 35 | 43 | 58 | 0.50 ms |
| 2.0 m | 518 | 40 | 49 | 64 | 1.26 ms |
| 4.0 m | 533 | 37 | 50 | 76 | 3.08 ms |

- A larger radius lets the lazy search solve more (478 to 533 pairs), at six to ten times the time. With the fallback on, those pairs are solved anyway and faster (`../ablation_charge_radius/`), so the radius is not a good lever for success.

## Limit

One map per configuration and a single seed for the start/goal pairs.
