# Ablation: other-side retry (planner v2.0.0)

When a waypoint cannot be placed, the planner can back up and try the side it
did not take. Five variants of LCP, all with the grid fallback **off** so that
the lazy search is measured alone, plus NavFn, on eight maps, 100 pairs each,
every pair planned 3 times. Full tables: `tables.md`.

| Variant | Retry | Order | Attempt budget |
|---|---|---|---|
| `noretry` | off | - | 300 |
| `recent300` | on | most recent decision first | 300 |
| `shortest100` | on | shortest partial path first | 100 |
| `shortest300` | on | shortest partial path first (default) | 300 |
| `shortest1000` | on | shortest partial path first | 1000 |

## Findings

Routable pairs solved by the lazy search alone, over all eight maps (704):

| Variant | Solved | Mean of the per-map median time | Slowest plan |
|---|---|---|---|
| `noretry` | 543 (77 %) | 1.49 ms | 26 ms |
| `recent300` | 555 (79 %) | 1.58 ms | 27 ms |
| `shortest100` | 531 (75 %) | 1.45 ms | 10 ms |
| `shortest300` (default) | 582 (83 %) | 1.62 ms | 27 ms |
| `shortest1000` | 627 (89 %) | 2.15 ms | 50 ms |

- Retrying the shortest partial path first beats retrying the most recent
  decision at the same budget (582 against 555), and both beat no retry (543).
- The budget matters more than the order: 100 attempts solve 531, 300 solve 582
  and 1000 solve 627, at the cost of a slowest plan of 10, 27 and 50 ms.
- The gains are on the hard maps. On the maze: 64 without retry, 82 with the
  default, 94 with a budget of 1000. On the open maps (base, corridors) every
  variant solves 97 to 100.
- Even the best variant leaves 11 % unsolved without the fallback: 63 of 94 at
  30 % density and 66 of 98 on the U shapes. This is what the grid fallback is
  for; with it every one of these pairs is solved (see `../ablation_fallback_budget/`).

## Limit

One map per configuration and a single seed for the start/goal pairs.
