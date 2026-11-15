# Ablation: force law, lazy search alone (planner v2.0.0)

The same five push rules as `../ablation_force_law/`, with the grid fallback
switched **off** in every variant, so the numbers show what each rule lets the
lazy search solve by itself. Eight maps, 100 pairs each, every pair planned 3
times. Full tables: `tables.md`.

| Variant | Push rule |
|---|---|
| `probe` | Legacy rule: push towards whichever side reaches free space sooner |
| `r1` | Coulomb sum with magnitude 1/r, first side kept |
| `r2` | Coulomb sum with magnitude 1/r², first side kept (the default) |
| `r1_free`, `r2_free` | As `r1`, `r2`, but the force may reverse the waypoint on every step |

## Findings

Routable pairs solved by the lazy search alone, over all eight maps (704):

| Variant | Solved | On the maze (100) | 30 % density (94) | U shapes (98) |
|---|---|---|---|---|
| `r2` (default) | 582 (83 %) | 82 | 51 | 53 |
| `r1` | 539 (77 %) | 61 | 41 | 46 |
| `r2_free` | 518 (74 %) | 55 | 38 | 48 |
| `r1_free` | 511 (73 %) | 52 | 35 | 45 |
| `probe` | 505 (72 %) | 51 | 35 | 40 |

- **1/r² with the first side kept is the best rule** on every hard map, and
  solves 43 more pairs than 1/r and 77 more than the legacy probe.
- **Both Coulomb laws beat the probe** once the first side is kept (582 and 539
  against 505). The summed force picks the better side more often than
  "whichever side is nearer".
- **Keeping the first side matters as much as the law**: letting the force
  reverse the waypoint costs 1/r² 64 pairs and 1/r 28, because the waypoint
  oscillates between obstacles.
- On the open maps (base, corridors, house) all rules solve 95 to 100 %; the
  differences are entirely on the cluttered ones.
- The probe is the cheapest per plan (median 0.5 to 1.7 ms against 1.3 to 3.1 ms
  for `r2`), but it solves fewer pairs.

This is the measurement behind the choice of 1/r² as the default, on the frozen
planner.

## Limit

One map per configuration and a single seed for the start/goal pairs.
