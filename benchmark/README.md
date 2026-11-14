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
