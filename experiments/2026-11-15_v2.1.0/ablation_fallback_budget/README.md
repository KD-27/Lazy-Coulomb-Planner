# Ablation: grid fallback and attempt budget (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

Variants: the lazy search alone (fallback off, budget 50), and the fallback on with budgets of 10, 25, 50 (default), 100 and 300 attempts.

## Findings

| Variant | Solved (of 704) | Mean of median | 95th pct | Slowest | Longest time to report no route |
|---|---|---|---|---|---|
| lazy search alone | 489 | 0.50 ms | 1.8 ms | 3.9 ms | 23 ms |
| fallback, budget 10 | 704 | 1.25 ms | 2.8 ms | 8.9 ms | 22 ms |
| fallback, budget 25 | 704 | 1.14 ms | 2.9 ms | 8.9 ms | 20 ms |
| fallback, budget 50 (default) | 704 | 1.23 ms | 3.6 ms | 8.9 ms | 24 ms |
| fallback, budget 100 | 704 | 1.41 ms | 4.5 ms | 12.1 ms | 21 ms |
| fallback, budget 300 | 704 | 1.62 ms | 6.9 ms | 20.6 ms | 22 ms |
| NavFn, for comparison | 682 | 8.58 ms | 15.5 ms | 28.8 ms | 35 ms |

- The fallback makes the planner complete on these maps (704 against 489).
- With the v2.1 A* fallback, budgets from 10 to 50 perform alike; larger budgets only make the slow cases slower (slowest plan 8.9 ms at 50, 20.6 ms at 300). 25 is marginally the best here; 50 keeps more pairs on the lazy path (see `../ablation_retry/`) for nearly the same time.
- Reporting that no route exists takes at most about 20 ms for every variant, faster than NavFn.

## Limit

One map per configuration and a single seed for the start/goal pairs.
