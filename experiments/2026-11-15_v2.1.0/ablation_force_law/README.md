# Ablation: force law (planner v2.1.0)

Eight maps (five generated: base, 20 % and 30 % density, concave U shapes, 1.2 m corridors; three indoor: maze, house, event venue), 100 pairs each, every pair planned 3 times, ground-truth routability, 704 routable pairs in all. Run on 15 November 2026 on `v2.1.0` compiled optimised. Every variant is the default LCP with only the listed parameter changed. Full tables: `tables.md`.

All variants keep the grid fallback and solve every routable pair (704 of 704), so the differences are in time and path shape.

## Findings

| Variant | Push rule | Mean of median time | 95th pct | Slowest |
|---|---|---|---|---|
| `probe` | Legacy: towards whichever side reaches free space sooner | 0.84 ms | 2.5 ms | 8.3 ms |
| `r1` | Coulomb sum 1/r, first side kept | 1.16 ms | 3.1 ms | 7.8 ms |
| `r2` | Coulomb sum 1/r², first side kept (default) | 1.23 ms | 3.7 ms | 10.4 ms |
| `r1_free` | 1/r, force may reverse the waypoint | 2.04 ms | 5.9 ms | 14.2 ms |
| `r2_free` | 1/r², force may reverse | 2.77 ms | 8.2 ms | 25.3 ms |

- In v2.1 the probe is the fastest overall. With a budget of only 50 attempts the lazy search hands hard cases to the A* fallback early, and the probe, being cheaper per step, wins on time even though it solves fewer pairs by itself (see `../ablation_force_law_lazy/`).
- Letting the force reverse the waypoint is still clearly worse: two to three times slower.
- Path length, clearance and smoothness are the same for all rules.

## Limit

One map per configuration and a single seed for the start/goal pairs.
