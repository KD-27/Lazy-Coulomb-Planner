# Ablation: force law, lazy search alone (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

The grid fallback is **off** in every variant, so the numbers show what the lazy search solves by itself.

## Findings

| Variant | Solved by the lazy search (of 704) | 30 % density (94) | U shapes (98) | Maze (100) |
|---|---|---|---|---|
| `r2` (default) | 489 (69 %) | 35 | 43 | 58 |
| `r1` | 489 (69 %) | 38 | 41 | 50 |
| `r1_free` | 454 (64 %) | 29 | 37 | 44 |
| `probe` | 449 (64 %) | 26 | 30 | 43 |
| `r2_free` | 443 (63 %) | 22 | 38 | 46 |

- Both Coulomb laws with the first side kept solve 40 more pairs than the legacy probe (489 against 449); 1/r² is ahead on the maze and U shapes, 1/r on the 30 % map.
- With the v2.1 budget of 50 attempts the gap between 1/r and 1/r² has closed (it was 582 against 539 at 300 attempts on v2.0.0). The choice of law matters less than keeping the first side, which is worth 35 to 46 pairs.
- The probe solves fewer pairs but is the fastest per plan; with the fallback on that makes it the fastest overall (`../ablation_force_law/`). For this planner the Coulomb force is a modest, measurable improvement in how often the cheap path succeeds, not the source of the speed.

## Limit

One map per configuration and a single seed for the start/goal pairs.
