# Phase 3 — Benchmark Harness Deployment

**Status:** ✅ Complete (May 5, 2026)

## Goal

Deploy the bundled `lcp_benchmark` ROS2 package, fix any deployment issues,
and run smoke tests for both NavFn (pipeline validation) and LazyCoulomb
(plugin-through-harness validation) on the `empty` map.

## Self-verification checklist (per protocol §3.8)

- [x] `lcp_benchmark` package builds without errors (after `setup.cfg` fix — see below)
- [x] LCP plugin name updated to `::` form in both config files
- [x] Map name `warehouse` → `house` in `experiment_grid.yaml`
- [x] Stale `outdoor_park` entry removed from `experiment_grid.yaml` (Phase 2 dropped Map 5)
- [x] Smoke test produces a CSV with exactly 3 rows (NavFn)
- [x] All 21 metric columns populated correctly
- [x] Manually inspected paths look reasonable on the map (`smoke_path_check.png`)
- [x] CSV opens cleanly in pandas
- [x] **Bonus:** LCP-through-harness verified on `empty` (success rate 100%, 154 waypoints — matches Phase 1)

## Files in this folder

| File | Purpose |
|---|---|
| `README.md` | This file |
| `smoke_test_navfn.csv` | NavFn × empty × {seed=0,1,2}. 3 rows, all success. |
| `smoke_test_lcp.csv` | LazyCoulomb × empty × {seed=0,1,2}. 3 rows, all success, 154 waypoints/run. |
| `smoke_seeds.yaml` | The 3 (start, goal) pairs used by both smoke tests (identical seeds → fair comparison). |
| `smoke_path_check.png` | Visual sanity check — start/goal markers overlaid on `empty.pgm`, with path lengths labelled. |
| `visual_check.py` | The script that produced `smoke_path_check.png`. Re-runnable for future smoke tests. |
| `harness_changes/` | Snapshot of files we created or edited inside `lcp_benchmark/`. See "Reproducibility" below. |

## Key results (NavFn vs LazyCoulomb on empty, 3 seeds)

| Metric | NavFn | LazyCoulomb |
|---|---|---|
| Success rate | 100% | 100% |
| Mean planning time | 16.2 ms | 7.8 ms |
| Path efficiency (mean) | 1.0034 | 1.0000 |
| Waypoints | 440–577 | 154 (constant) |
| Curvature | 0.4–4.2 | 0.0 (straight lines on empty) |

LCP returns the straight-line baseline + Chaikin smoothing on empty — exactly as designed.
NavFn produces denser, slightly curvier paths around obstacles.

## Issues found and fixed

1. **`lcp_benchmark` was missing `setup.cfg`.** `colcon build` reported success but `ros2 run` failed with "No executable found." Console scripts were installed to `install/<pkg>/bin/` instead of the ROS2-expected `install/<pkg>/lib/<pkg>/`. Added a standard `setup.cfg` with `[develop] script_dir` and `[install] install_scripts` both pointing to `$base/lib/lcp_benchmark`. See `harness_changes/setup.cfg`.

2. **`experiment_grid.yaml` had stale `outdoor_park` map entry.** Phase 2 dropped Map 5 (`outdoor_park`), but the bundled harness was generated under the original 5-map plan. Removed the entry, updated header comment from `4 * 5 * 30 = 600 runs` to `4 * 4 * 30 = 480 runs`.

3. **LCP plugin name mismatch.** Bundled config used the slash form (`lazy_coulomb_planner/LazyCoulombPlanner`), but the plugin registers under the `::` C++ namespace form (Deviation #2 in master protocol). Updated both `experiment_grid.yaml` and `nav2_params_lazy_coulomb.yaml`.

4. **Map name `warehouse` → `house`.** Per Deviation #1, AWS warehouse repo failed to build, replaced by AWS small_house. Updated map references in `experiment_grid.yaml`.

## Watch items for Phase 5+

- **LCP curvature = 0.0 on straight-line paths is mathematically correct.** But verify it becomes nonzero on cluttered maps (`maze`, `event_venue`, `house`) — if it stays zero, there is a bug in `metrics.py`'s curvature calculation.
- **`planning_time_ms` measures the full `ComputePathToPose` action call (including ROS2 IPC overhead),** not just the planner's internal compute. This is the right metric for a Nav2 user-facing benchmark, but should be made explicit in the paper's Section 5 methodology.
- **`cpu_percent_avg = 0.0` for all 3 NavFn rows and all 3 LCP rows on empty.** Likely because runs are too fast (5–20 ms) for the 50 ms psutil polling interval to catch a sample. Should populate on slower planners and harder maps. If it stays 0.0 across the full sweep, tighten the polling interval.

## Reproducibility

Starting from a clean clone of the LCP planner repo (this repo) plus `lcp_benchmark.zip` (the bundled harness from the protocol):

```bash
# 1. Unzip the harness
cd ~/lcp_paper_ws/src/
unzip /path/to/lcp_benchmark.zip

# 2. Apply the patches snapshotted in this folder
cp evaluation/phase3/harness_changes/setup.cfg                            ~/lcp_paper_ws/src/lcp_benchmark/setup.cfg
cp evaluation/phase3/harness_changes/experiment_grid.yaml                 ~/lcp_paper_ws/src/lcp_benchmark/config/
cp evaluation/phase3/harness_changes/nav2_params_lazy_coulomb.yaml        ~/lcp_paper_ws/src/lcp_benchmark/config/
cp evaluation/phase3/harness_changes/smoke_test_lcp.launch.py             ~/lcp_paper_ws/src/lcp_benchmark/launch/

# 3. Build
cd ~/lcp_paper_ws
colcon build --packages-select lcp_benchmark --symlink-install
source install/setup.bash

# 4. Run smoke tests
ros2 launch lcp_benchmark smoke_test.launch.py        # NavFn
ros2 launch lcp_benchmark smoke_test_lcp.launch.py    # LCP
```
