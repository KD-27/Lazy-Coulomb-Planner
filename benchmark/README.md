# Benchmark

Compares the Lazy Coulomb Planner with Nav2's grid planners on a shared global
costmap, without a simulator: only `map_server` and `planner_server` run.

It is derived from Nav2's `tools/planner_benchmarking`, with three changes:
failed requests are recorded instead of discarded, every returned path is
audited against the costmap at a tenth of a cell, and the map origin is
respected.

## Files

| File | Purpose |
|---|---|
| `Dockerfile` | ROS 2 Jazzy + full Nav2 image, for machines without a native install |
| `run_all.sh` | All six maps |
| `bench.sh` | One map; run where Nav2 and this workspace are available |
| `lcp_bench.py` | Samples start/goal pairs, calls the planners, writes a CSV and a summary |
| `config/planners.yaml` | Planner plugins and the costmap (0.22 m robot radius, 0.7 m inflation) |
| `maps/` | `event_venue`, `maze`, `house`, `empty`; `fetch_nav2_maps.sh` adds Nav2's 100 m maps |
| `results/` | CSVs of the last full run |

## Suites

`run_all.sh` covers the six fixed maps. For controlled experiments use the
suite runner, which generates its maps and collects one summary:

| File | Purpose |
|---|---|
| `generate_map.py` | Map with a chosen size, obstacle density and shape (circles, squares, concave U, or a block grid with corridors), reproducible from a seed |
| `run_suite.py` | Runs a suite: generates each map, benchmarks every planner on it, writes `summary.csv`; resumable |
| `suites/controlled.yaml`, `suites/controlled_seeds.yaml` | Density, size, shape and corridor-width sweeps, and their repeats with other seeds |
| `suites/ablation/` | One suite per LCP design choice, with the planner variants it compares |
| `routability.py` | Ground truth for which pairs have a route, computed from the map; re-scores every planner against it |
| `report_suite.py` | Tables (Markdown) and plots from a finished suite |
| `aggregate_seeds.py` | Mean and spread over suites that repeat configurations with different seeds |

```bash
benchmark/run_suite.py benchmark/suites/controlled.yaml <out_dir>
benchmark/routability.py <out_dir>
benchmark/report_suite.py <out_dir> "Controlled benchmark"
```

Set `LCP_BENCH_REPEATS` (the suites do) to plan every pair several times and
report the median time.

## Running

```bash
colcon build --packages-select lazy_coulomb_planner      # from the workspace root
benchmark/run_all.sh final 100 Navfn Smac2d ThetaStar LCP
```

This needs Nav2 Jazzy (planner, map server, lifecycle manager, simple commander
and the three grid planners). Without a native install, build the image with
`docker build -t lcp-jazzy benchmark/`, mount the workspace at the same path it
has on the host, and run the same two commands inside it.

Pairs are drawn with a fixed seed from cells with cost below 210, at least 3 m
apart. The first planner named is the reference: a pair counts as having a
route if that planner returns a collision-free path.

## Summary columns

| Column | Meaning |
|---|---|
| `valid` | Collision-free paths / pairs that have a route |
| `collide` | Paths returned that touch a cell with cost 253 or more |
| `failed` | Pairs with a route for which the planner reported failure |
| `false+` | Paths returned for pairs without a route |
| `t_med`, `t_p95`, `t_max` | Planning time reported by `planner_server`, returned paths only (ms) |
| `failwall` | Slowest failed request, client round trip (ms) |
| `len/ref` | Median path length relative to the reference planner |
| `turn` | Median total heading change (rad) |
| `corner` | Median sharpest single heading change (degrees) |
| `maxcost`, `meancost` | Median of the highest / average costmap cost along the path (lower = further from obstacles) |
