# Ablation: other-side retry and budget (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

The grid fallback is **off** in every variant, so the numbers show what the lazy search solves by itself.

## Findings

| Variant | Solved by the lazy search (of 704) | 30 % density | U shapes | Maze | Mean of median | Slowest |
|---|---|---|---|---|---|---|
| no retry | 479 | 35 | 42 | 52 | 0.50 ms | 3.6 ms |
| most recent first, 50 | 487 | 35 | 43 | 58 | 0.50 ms | 5.6 ms |
| shortest first, 25 | 405 | 23 | 33 | 51 | 0.39 ms | 1.8 ms |
| shortest first, 50 (default) | 489 | 35 | 43 | 58 | 0.50 ms | 3.9 ms |
| shortest first, 100 | 537 | 43 | 46 | 73 | 0.55 ms | 6.4 ms |
| shortest first, 300 | 601 | 56 | 58 | 90 | 0.79 ms | 14.5 ms |

- With only 50 attempts the retry and its order hardly matter (479 to 489 pairs): there are few attempts to spend on alternatives. The budget decides: 25 attempts solve 405, 300 solve 601.
- With the fallback on, a smaller budget is faster overall (`../ablation_fallback_budget/`), which is why the default is 50 even though the lazy search alone would solve more with a larger one.

## Limit

One map per configuration and a single seed for the start/goal pairs.
