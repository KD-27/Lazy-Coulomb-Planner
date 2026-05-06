# Phase 4 — LCP Parameter Tuning

Tuning sweep over LCP's three primary parameters on the `event_venue` map, used
to pick defaults before the main 480-run benchmark in Phase 5.

## Sweep design

- **Map:** `event_venue` only (per master protocol §4.2)
- **Grid:** 3 × 3 × 3 = 27 parameter combinations
  - `step_size` ∈ {0.02, 0.05, 0.10}
  - `repulsion_strength` ∈ {0.05, 0.15, 0.30}
  - `initial_path_points` ∈ {10, 20, 40}
- **Seeds per combo:** 10 (same 10 (start, goal) pairs replayed across all
  combos, so timing differences reflect parameter effects, not scenario noise)
- **Total runs:** 270 (~6m 29s wall clock; protocol estimated ~30 min)

## Results summary

- All 27 combos achieved 100% success rate. Zero combos filtered by §4.3 step 1.
- `initial_path_points` is the dominant cost driver:
  - All 9 `pts=10` combos: mean times 2.54–3.08 ms
  - All 9 `pts=20` combos: mean times 3.98–4.44 ms
  - All 9 `pts=40` combos: mean times 6.53–7.83 ms
- Within each `pts` group, `step_size` and `repulsion_strength` show no
  detectable effect (within-group variation ≈ within-combo stddev).

## Chosen defaults

| Parameter | Pre-Phase-4 | Strict §4.3 winner | Chosen |
|---|---|---|---|
| `step_size` | 0.05 | 0.05 | **0.05** |
| `repulsion_strength` | 0.15 | 0.05 | **0.15** |
| `initial_path_points` | 20 | 10 | **10** |

The strict §4.3 winner `(0.05, 0.05, 10)` was overridden in favor of
`(0.05, 0.15, 10)`. Rationale:

1. The 0.54 ms gap to rank-9 is below within-combo stddev — picking by literal
   mean tunes to seed-to-seed noise.
2. `repulsion_strength=0.05` is the lowest grid point. Edge-of-grid values are
   less defensible as published defaults; mid-grid `0.15` is more conservative.
3. `pts=10` is also the lowest grid point but kept because it is the only
   parameter with a real timing effect.
4. Net change vs. pre-tuning: only `initial_path_points: 20 → 10`.

## Layout

```
phase4/
├── README.md                 # this file
├── chosen_config.yaml        # post-tuning nav2_params_lazy_coulomb.yaml
├── tuning_sweep.csv          # 270 rows × 24 columns (raw harness columns + 3 added)
├── sweep_configs/            # the 27 generated YAMLs (one per combo)
│   └── sweep_s{step}_r{rep}_p{pts}.yaml × 27
└── tools/                    # Phase-4-specific scripts
    ├── generate_sweep_configs.py    # base YAML → 27 templated YAMLs
    ├── run_tuning_sweep.sh          # full driver: preflight + 270 runs
    └── merge_tuning_csvs.py         # 27 per-combo CSVs → tuning_sweep.csv
```

## Reproducibility

To rerun the sweep from a fresh checkout:

1. Deploy the harness package per Phase 3 instructions.
2. Apply the chosen config: `cp evaluation/phase4/chosen_config.yaml ~/lcp_paper_ws/src/lcp_benchmark/config/nav2_params_lazy_coulomb.yaml`
3. Source workspace, then: `evaluation/phase4/tools/run_tuning_sweep.sh`
4. The merged `tuning_sweep.csv` should match the snapshotted file modulo seed-to-seed noise.

The 27 per-combo CSVs produced by `run_tuning_sweep.sh` are intermediate; they
were not snapshotted because all rows are recoverable from `tuning_sweep.csv`
by filtering on the three added columns (`step_size`, `repulsion_strength`,
`initial_path_points`).

## Watch items / paper-relevant findings

- **`metrics.py` curvature returns 0.0 on cluttered maps.** Phase 3 watch-item
  #1 escalated to confirmed bug. Across all 270 runs on `event_venue`,
  `cumulative_curvature = 0.0` despite paths having 314 waypoints weaving
  around obstacles. Mathematically implausible. **Must be fixed before Phase 6
  statistical analysis.**
- **`repulsion_strength` does not control final path clearance** — only
  per-step push magnitude during escape. Mean `min_clearance_m` is essentially
  identical across all `repulsion_strength` values within each `pts` group.
  Paper Discussion section material; not a bug.

## Tools used (process / tooling lessons recorded for Phase 5+)

- `setsid` + process-group teardown is required for `ros2 launch` (parent SIGINT does not cascade to children).
- Strict precondition check at script start prevents stale Nav2 processes from silently shadowing new bringups.
- Per-combo CSV + post-merge is cleaner than mid-protocol harness extension.
