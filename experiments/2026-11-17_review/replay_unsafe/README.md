# Replay of the two fixed-side paths that touched cost 253

In `../ablation_side_rule/`, `fixed_left` (LCP with every push to the left first, fallback on)
returned two paths whose audit found a sample in a cell of cost 253 (inscribed zone):
density_20 pair 64 (length 35.906118 m) and density_30 pair 88 (26.977742 m). In both, the
lazy search alone failed (`fixed_left_lazy`), so the path came from the A* fallback plus
post-processing. Default LCP has never returned such a path (6234 + 1967 + 704 routable pairs).

Replays on 17 November 2026, ROS domain 88, while the MPPI driving runs used Docker:

1. `run.sh` / `replay.py`: the two requests alone, for fixed_left, coulomb and probe: every
   path clean (no sample in a cell >= 253). Outputs `*_*.json` (paths and touched cells).
2. `bench.sh` with the same planners file, pairs 0-88 in the original order, 3 repeats,
   planners fixed_left and coulomb (`seq_*.csv`): every path clean. Pair 64 on density_20:
   fixed_left length 35.906001 m, max cost 234; pair 88 on density_30: 26.978834 m, max cost 234.

The replayed paths are the original ones to within 0.1 to 1 mm in length, but their highest
cost is now 234, not 253. The original paths therefore ran along the edge of an
inscribed-zone cell, and a sub-millimetre difference in the returned coordinates decided
whether one audit sample (spaced a tenth of a cell) fell just inside it. This is a
floating-point boundary case at a cell edge, not a path through the inscribed zone, and it
does not reproduce. Why the coordinates differ by a fraction of a millimetre between runs was
not traced (planner source unchanged).
