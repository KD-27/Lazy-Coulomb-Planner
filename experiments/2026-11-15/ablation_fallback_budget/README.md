# Ablation: grid fallback and attempt budget (planner v2.0.0)

When the lazy search uses up its attempt budget, LCP can hand over to a grid
search. Five variants: the lazy search alone, and the fallback on with budgets
of 100, 300 (default), 1000 and 2000 attempts; plus NavFn. Eight maps, 100 pairs
each, every pair planned 3 times. Full tables: `tables.md`.

## Findings

| Variant | Routable pairs solved (of 704) | Mean of the per-map median time | Mean 95th percentile | Slowest plan |
|---|---|---|---|---|
| Lazy search alone, budget 300 | 582 (83 %) | 1.7 ms | 9.9 ms | 27 ms |
| Fallback, budget 100 | 704 | 6.0 ms | 21.7 ms | 58 ms |
| Fallback, budget 300 (default) | 704 | 5.2 ms | 26.3 ms | 66 ms |
| Fallback, budget 1000 | 704 | 5.0 ms | 36.8 ms | 107 ms |
| Fallback, budget 2000 | 704 | 5.1 ms | 50.8 ms | 179 ms |

- The fallback is what makes LCP complete on these maps: the lazy search alone
  solves 83 % of the routable pairs, with the fallback every one. The pairs it
  rescues are on the hard maps (30 % density 51 of 94 alone, U shapes 53 of 98,
  maze 82 of 100).
- Completeness costs time on exactly those maps. On the 30 % map the median goes
  from 3 ms (lazy alone, 51 solved) to 19 ms (94 solved); on easy maps the
  fallback is never used and costs nothing.
- The budget trades typical time against worst-case time, not success. A larger
  budget lets the lazy search solve more by itself, which lowers the median a
  little, but the slowest plan grows from 58 ms at 100 attempts to 179 ms at
  2000, because a request that will fail anyway spends the whole budget first.
- The default of 300 sits in the middle. A budget of 100 has the best worst
  case with only slightly higher medians and marginally shorter paths.

## Limit

One map per configuration and a single seed for the start/goal pairs.
