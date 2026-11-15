# Ablation: charge radius, lazy search alone (planner v2.0.0)

The same four radii as `../ablation_charge_radius/` (0.5, 1.0 = default, 2.0,
4.0 m) with the grid fallback switched **off**, so the numbers show what each
radius lets the lazy search solve by itself. Eight maps, 100 pairs each, every
pair planned 3 times. Full tables: `tables.md`.

## Findings

| Radius | Routable pairs solved (of 704) | Maze (100) | 30 % density (94) | U shapes (98) |
|---|---|---|---|---|
| 0.5 m | 572 (81 %) | 80 | 50 | 56 |
| 1.0 m (default) | 582 (83 %) | 82 | 51 | 53 |
| 2.0 m | 595 (85 %) | 86 | 57 | 55 |
| 4.0 m | 599 (85 %) | 84 | 58 | 58 |

- A larger radius helps the lazy search a little: from 0.5 m to 4 m it solves 27
  more pairs of 704 (81 % to 85 %). Most of the gain is already there at 2 m.
- It is expensive. On the 30 % density map the median time goes from 2.1 ms at
  0.5 m to 24 ms at 4 m for 8 more pairs solved.
- With the grid fallback on, every one of these pairs is solved anyway
  (`../ablation_charge_radius/`), and there a small radius is simply faster. So
  the radius is not a good lever for success: the fallback does that job far
  more cheaply.

## Limit

One map per configuration and a single seed for the start/goal pairs.
