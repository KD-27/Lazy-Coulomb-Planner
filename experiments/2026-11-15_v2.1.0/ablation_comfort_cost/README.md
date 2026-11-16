# Ablation: comfort cost (planner v2.1.0)

The costmap cost above which the planner tries to back away from obstacles; 253
switches the clearance steps off. Six variants of LCP (off, 150, 100, 60, 30 =
default, 1) plus NavFn, on the eight ablation maps, 100 pairs each, every pair
planned 3 times. Full tables: `tables.md`. The first run of this suite crashed
because the variant name `off` was read by YAML as a boolean; this is the rerun
of 15 November 2026, 22:45 to 22:52.

All variants solve every routable pair (704 of 704).

| Comfort cost | Mean cost along the path | Worst cost on a typical path | Length vs NavFn | Mean of median time |
|---|---|---|---|---|
| off (253) | 78.0 | 234 | 0.938 | 1.04 ms |
| 150 | 65.9 | 193 | 0.948 | 1.12 ms |
| 100 | 55.1 | 175 | 0.958 | 1.18 ms |
| 60 | 40.5 | 163 | 0.971 | 1.23 ms |
| 30 (default) | 39.2 | 164 | 0.972 | 1.21 ms |
| 1 | 39.1 | 165 | 0.972 | 1.25 ms |
| NavFn | 15.3 | 127 | 1.000 | 8.57 ms |

## Findings

- The clearance steps halve the average cost along the path (78 to 39) for 3 %
  more length and about 15 % more time.
- The effect saturates at 60: going lower changes nothing measurable. The default
  of 30 is in the saturated range.
- LCP's paths stay closer to obstacles than NavFn's (39 against 15), as in
  v2.0.0: the clearance steps move the path away from the obstacles it goes
  around, but do not centre it in free space.

## Limit

One map per configuration and a single seed for the start/goal pairs.
