# Ablation: simplification and smoothing (planner v2.0.0)

What happens to the path after it is found. Six variants of LCP: `raw` (no
waypoint simplification, no smoothing), `no_smooth` (simplification only), and
1, 3, 5 (default) or 8 Chaikin passes; plus NavFn. Eight maps, 100 pairs each,
every pair planned 3 times. Full tables: `tables.md`.

## Findings

All variants solve every routable pair (704 of 704): no smoothing setting ever
produced a path that touches an obstacle.

| Variant | Length vs NavFn | Total turning | Sharpest corner, median | Sharpest corner, 90th pct | Mean cost | Mean of median time |
|---|---|---|---|---|---|---|
| `raw` | 0.997 | 8.3 rad | 70° | 179° | 31.8 | 5.4 ms |
| `no_smooth` | 0.983 | 5.2 rad | 59° | 163° | 35.0 | 4.6 ms |
| 1 pass | 0.978 | 5.2 rad | 46° | 155° | 38.2 | 4.5 ms |
| 3 passes | 0.975 | 5.2 rad | 34° | 142° | 39.0 | 4.6 ms |
| 5 passes (default) | 0.975 | 5.2 rad | 31° | 133° | 39.8 | 4.9 ms |
| 8 passes | 0.975 | 5.2 rad | 30° | 132° | 45.2 | 7.0 ms |

"Sharpest corner" is the largest change of heading between two consecutive
output segments (poses are 0.1 m apart), per path.

- Removing redundant waypoints is the bigger step: it cuts total turning by a
  third (8.3 to 5.2 rad) and shortens paths by 1.4 %.
- Smoothing then spreads the remaining turns: the typical sharpest corner goes
  from 59° to 31° with five passes. Total turning stays the same, as it should.
- Returns diminish after three passes. Eight passes give nothing more in
  corners and cost 40 % more time and closer paths (cost 45 against 40).
- Smoothing brings paths slightly closer to obstacles (cost 35 without, 40 with
  five passes), because a rounded corner cuts inside the sharp one.

## A weakness this exposes

Even with five passes, one path in ten still has a corner sharper than 130°, a
near reversal. Smoothing only reduces it from 163° to 133°. These are not
rounded because a corner is cut only as far as the cut stays clear of obstacles
and within the allowed cost; where that is not possible it is kept. They are
concentrated on the hard maps (30 % density, U shapes, maze), where many paths
come from the grid fallback. This is a path-quality problem in v2.0.0 that the
planner benchmark's other metrics do not show, and a candidate for the next
planner change.

## Limit

One map per configuration and a single seed for the start/goal pairs.
