# Ablation: charge radius (planner v2.0.0)

The range within which obstacle cells repel a waypoint: 0.5, 1.0 (default), 2.0
and 4.0 m. Four variants of LCP that differ only in this, plus NavFn, on eight
maps, 100 pairs each, every pair planned 3 times. Full tables: `tables.md`.

## Findings

All variants keep the grid fallback and solve every routable pair (704 of 704),
so the difference is in time and in how close paths run to obstacles.

| Radius | Mean of the per-map median time | Slowest plan | Mean cost along the path |
|---|---|---|---|
| 0.5 m | 3.1 ms | 60 ms | 43.3 |
| 1.0 m (default) | 5.0 ms | 67 ms | 39.8 |
| 2.0 m | 8.2 ms | 104 ms | 39.6 |
| 4.0 m | 18.5 ms | 318 ms | 39.3 |

- Time rises steeply with the radius, as expected: the force sum visits a
  number of cells that grows with the square of the radius. From 1 m to 4 m the
  average time is 3.7 times higher and the slowest plan almost 5 times.
- A larger radius buys almost nothing here: path length is unchanged and the
  average cost along the path improves by about 1 % from 1 m to 4 m.
- 0.5 m is the fastest, by about 40 % over the default, but its paths run
  closer to obstacles (cost 43.3 against 39.8), most visibly on the 20 %
  density map (56.5 against 48.5).

The default of 1.0 m is a reasonable middle; 0.5 m is defensible where speed
matters more than clearance.

## Limit

Whether a larger radius helps the lazy search succeed more often is hidden by
the fallback here. The same comparison with the fallback off is in
`../ablation_charge_radius_lazy/`.
