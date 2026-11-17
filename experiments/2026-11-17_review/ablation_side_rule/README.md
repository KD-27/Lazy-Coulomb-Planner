# Ablation: side rule, all variants in one run (planner v2.1.0)

The eight ablation maps (50 m at 10, 20, 30 %, U shapes, 1.2 m corridors; maze, house,
event venue), 100 pairs, 3 repeats, ground-truth routability (704 routable pairs), 17 November
2026. Suite: `benchmark/suites/ablation/side_rule.yaml`. Variants: Coulomb 1/d^2 (default),
fixed side (`force_balance_threshold: 1e9`, every push left first), nearest-free-side probe
(`force_model: probe`), each with the fallback on and off (`*_lazy`), and the lazy search
alone at budget 10 (`coulomb_lazy_b10`). One run, so no run-to-run noise between rows.

| Rule | Solved without fallback (of 704) | Mean of median time, fallback on (ms) | Mean p95 (ms) |
|---|---|---|---|
| Coulomb (default) | 489 | 1.21 | 3.8 |
| probe | 449 | 0.81 | 2.4 |
| fixed side | 440 | 1.70 | 4.1 |
| Coulomb, budget 10 | 249 | - | - |

- Both informed rules beat a fixed side; the probe is about 1.5x faster than Coulomb with
  the same path length and cost (see `../offline/` for the per-pair comparison), Coulomb
  repairs more requests without the fallback. The counts 489 / 449 repeat the separate
  2026-11-15 runs exactly. Paper: LCP presented with both rules, Coulomb the default.
- fixed_left returned two paths whose audit found one sample in a cost-253 cell; replay shows
  these are cell-edge rounding cases, not paths through the inscribed zone
  (`../replay_unsafe/README.md`).
