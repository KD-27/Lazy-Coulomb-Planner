# Ablation: simplification and smoothing (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

All variants keep the grid fallback and solve every routable pair (704 of 704), so the differences are in time and path shape.

## Findings

| Variant | Paths with a corner over 90° (of 704) | Total turning | Length vs NavFn | Mean cost | Mean of median time |
|---|---|---|---|---|---|
| raw (no simplification, no smoothing) | 178 | 7.2 rad | 0.990 | 32.9 | 1.23 ms |
| simplification only | 50 | 5.0 rad | 0.981 | 35.1 | 1.14 ms |
| 1 Chaikin pass | 29 | 4.7 rad | 0.975 | 38.3 | 1.11 ms |
| 3 passes | 12 | 4.4 rad | 0.972 | 39.1 | 1.18 ms |
| 5 passes (default) | 11 | 4.4 rad | 0.972 | 39.2 | 1.22 ms |
| 8 passes | 11 | 4.4 rad | 0.972 | 39.3 | 1.54 ms |
| NavFn, for comparison | 187 | 20.7 rad | 1.000 | 15.3 | 8.53 ms |

- The farthest-visible simplification of v2.1 removes most sharp corners by itself (178 to 50) and cuts total turning by 30 %. Smoothing then rounds most of the rest (11 left after 5 passes, 1.6 %).
- Three passes are as good as five; eight only cost time.
- LCP paths turn far less than NavFn's (4.4 against 20.7 rad on average): NavFn follows the grid and the cost gradient, LCP draws straight segments.

## Limit

One map per configuration and a single seed for the start/goal pairs.
