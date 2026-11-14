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
| `Dockerfile` | ROS 2 Jazzy + Nav2 image (`docker build -t lcp-jazzy benchmark/`) |
| `run_all.sh` | Host script: all six maps, inside the image |
| `bench.sh` | One map; run where Nav2 and this workspace are available |
| `lcp_bench.py` | Samples start/goal pairs, calls the planners, writes a CSV and a summary |
| `config/planners.yaml` | Planner plugins and the costmap (0.22 m robot radius, 0.7 m inflation) |
| `maps/` | `event_venue`, `maze`, `house`, `empty`; `fetch_nav2_maps.sh` adds Nav2's 100 m maps |
| `results/` | CSVs of the last full run |

## Running

```bash
docker build -t lcp-jazzy benchmark/
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v <workspace>:/ws lcp-jazzy \
  bash -c 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build'
benchmark/run_all.sh final 100 Navfn Smac2d ThetaStar LCP
```

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
