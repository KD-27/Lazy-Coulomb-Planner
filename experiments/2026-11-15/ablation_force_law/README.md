# Ablation: force law (planner v2.0.0)

How a waypoint inside an obstacle is pushed out. Five variants of LCP that
differ only in this, plus NavFn as reference, on eight maps (five generated,
three indoor), 100 pairs each, every pair planned 3 times.

| Variant | Push rule |
|---|---|
| `probe` | Legacy rule: push towards whichever side reaches free space sooner |
| `r1` | Coulomb sum with magnitude 1/r, first side kept |
| `r2` | Coulomb sum with magnitude 1/r², first side kept (the default) |
| `r1_free`, `r2_free` | As `r1`, `r2`, but the force may reverse the waypoint on every step |

Full tables: `tables.md`. Ground truth: `summary_truth.csv`.

## Findings

All variants keep the grid fallback, so all of them solve every routable pair
(704 of 704). The fallback hides how often the lazy search itself fails; what
differs here is time, because a failed lazy search pays for the fallback.

| Variant | Mean of the per-map median time | Slowest plan |
|---|---|---|
| `r2` (default) | 5.1 ms | 65 ms |
| `probe` | 5.2 ms | 57 ms |
| `r1` | 7.7 ms | 73 ms |
| `r1_free` | 15.1 ms | 162 ms |
| `r2_free` | 20.7 ms | 220 ms |

- Letting the force reverse the waypoint (`*_free`) is three to four times
  slower overall and worst on the hard maps (30 % density: 58 and 93 ms against
  18 ms). The waypoint oscillates between obstacles, the push fails, and the
  fallback has to step in.
- With the first side kept, 1/r² is faster than 1/r on six of eight maps and
  about level with the probe overall. It is clearly better than both on the
  maze (2.7 ms against 4.9 and 5.6 ms) and the U shapes (12.3 against 17.3 and
  23.8 ms), and somewhat slower than the probe on the 30 % map (18.5 against
  14.6 ms).

## Limit

To see the push rules' effect on success directly, the same comparison with the
grid fallback off is in `../ablation_force_law_lazy/`.
