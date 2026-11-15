# Ablation: waypoint margin (planner v2.0.0)

How far a waypoint keeps moving past the edge of the obstacle it has just
cleared: 0, 0.15, 0.3 (default) and 0.5 m. Four variants of LCP that differ
only in this, plus NavFn, on eight maps, 100 pairs each, every pair planned 3
times. Full tables: `tables.md`.

## Findings

All variants keep the grid fallback and solve every routable pair (704 of 704),
so the difference is in time.

| Margin | Mean of the per-map median time | Slowest plan | Length vs NavFn | Mean cost along the path |
|---|---|---|---|---|
| 0 m | 8.4 ms | 61 ms | 0.974 | 37.6 |
| 0.15 m | 5.7 ms | 67 ms | 0.973 | 38.3 |
| 0.3 m (default) | 5.1 ms | 67 ms | 0.975 | 39.8 |
| 0.5 m | 4.2 ms | 68 ms | 0.976 | 39.7 |

- A margin makes planning faster: with none, the average time is 60 % higher
  than with the default. A waypoint left exactly on the obstacle's edge tends to
  need further corrections; one that is clear of it does not. The effect is
  largest on the cluttered maps (20 % density 5.7 ms against 2.2 ms, corridors
  5.5 against 1.9 ms, maze 6.2 against 2.6 ms).
- Path length barely changes (within 0.3 %).
- A larger margin does not buy clearance: the average cost along the path is
  about the same or slightly higher with a bigger margin, because the clearance
  steps that follow already move waypoints where they should be.
- 0.5 m is a little faster again than the default, with no measurable cost in
  length. The default could reasonably be raised.

## Limit

The margin's effect on how often the lazy search succeeds is hidden by the
fallback here. The same comparison with the fallback off is in
`../ablation_waypoint_margin_lazy/`.
