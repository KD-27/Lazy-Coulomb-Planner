# Ablation: comfort cost (planner v2.0.0)

The costmap cost above which the planner tries to back away from obstacles.
253 switches the clearance steps off; lower values ask for more distance. Six
variants of LCP that differ only in this (off, 150, 100, 60, 30 = default, 1),
plus NavFn, on eight maps, 100 pairs each, every pair planned 3 times. Full
tables: `tables.md`.

## Findings

All variants solve every routable pair (704 of 704). What changes is how close
paths run to obstacles, their length, and planning time. Averages over the
eight maps:

| Comfort cost | Mean cost along the path | Worst cost on a typical path | Length vs NavFn | Mean of the per-map median time |
|---|---|---|---|---|
| off (253) | 82.0 | 234 | 0.943 | 3.4 ms |
| 150 | 70.2 | 194 | 0.952 | 4.5 ms |
| 100 | 57.9 | 171 | 0.963 | 5.2 ms |
| 60 | 41.7 | 163 | 0.975 | 5.2 ms |
| 30 (default) | 39.8 | 161 | 0.975 | 5.0 ms |
| 1 | 40.0 | 162 | 0.975 | 5.1 ms |
| NavFn, for comparison | 15.3 | - | 1.000 | 8.6 ms |

- The clearance steps halve the average cost along the path (82 to 40) and bring
  the worst point of a typical path from cost 234, right at the edge of
  contact, down to about 160.
- The price is path length and time: paths are 3 % longer than with clearance
  off (still 2.5 % shorter than NavFn's), and planning takes about 50 % longer.
- The effect saturates at 60. Going lower (30, 1) changes nothing measurable:
  beyond that point the planner cannot move waypoints any further without
  giving up its other rules. The default of 30 is in the saturated range.
- LCP stays well short of NavFn's clearance on every map (mean cost 40 against
  15), most of all in open maps, where NavFn's cost is near zero and LCP's is
  around 20. LCP keeps away from the obstacles it has to go around but does not
  steer towards the middle of free space.
- On the densest map (30 %) the worst cost stays at 234 for every setting: there
  the gaps are too narrow for any choice.

## Limit

One map per configuration and a single seed for the start/goal pairs. The costs
depend on the costmap's inflation settings (radius 0.7 m, scaling 3.0).
