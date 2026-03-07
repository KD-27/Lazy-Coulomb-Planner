# Lazy Coulomb Planner — Nav2 Global Planner Plugin

A Nav2 global planner plugin for ROS 2 Humble that implements reactive path
planning inspired by **electrostatic repulsion (Coulomb's law)**.

**Developed by Kaveesha Dhananjaya**

---

## Algorithm Overview

The planner starts with the laziest possible assumption — a straight line from
start to goal — and only does work when that line is blocked.

```
1. Draw straight line  S ──────────────────── G
2. Find FIRST segment that clips an obstacle
3. Insert a waypoint at the entry point
4. Push it perpendicular to the path until clear
5. Lock it (it won't move again)
6. Repeat from step 2
7. Apply optional Chaikin smoothing
```

Each waypoint is pushed **perpendicular to the local path direction**, choosing
the side with the shorter distance to clear space. This ensures the detour
stays as close to the original straight line as possible.

---

## Package Structure

```
lazy_coulomb_planner/
├── CMakeLists.txt
├── package.xml
├── planner_plugins.xml          ← pluginlib registration
├── config/
│   └── nav2_params.yaml         ← drop-in Nav2 parameter file
├── include/lazy_coulomb_planner/
│   └── lazy_coulomb_planner.hpp
└── src/
    └── lazy_coulomb_planner.cpp
```

---

## Build & Install

```bash
# 1. Clone into your workspace
cd ~/ros2_ws/src
git clone <this_repo> lazy_coulomb_planner

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --packages-select lazy_coulomb_planner

# 4. Source
source install/setup.bash
```

---

## Activating the Plugin

In your `nav2_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "lazy_coulomb_planner::LazyCoulombPlanner"
      max_iterations: 1000
      step_size: 0.05
      repulsion_strength: 0.15
      enable_smoothing: true
      smoothing_iterations: 3
```

Then launch Nav2 pointing at your params file:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/path/to/nav2_params.yaml \
  use_sim_time:=true
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_iterations` | 1000 | Hard cap on total algorithm steps |
| `max_push_iterations` | 300 | Max steps to push one waypoint clear |
| `step_size` | 0.05 | Integration step size (meters) |
| `repulsion_strength` | 0.15 | Force magnitude per step |
| `force_balance_threshold` | 0.01 | Below this, use fallback direction |
| `perturbation_strength` | 0.08 | Fallback nudge magnitude |
| `initial_path_points` | 20 | Waypoints in initial straight line |
| `lethal_cost_threshold` | 253.0 | Costmap cost = obstacle boundary |
| `segment_check_steps` | 20 | Samples per segment for intersection check |
| `enable_smoothing` | true | Apply Chaikin smoothing to final path |
| `smoothing_iterations` | 3 | Chaikin passes (1–8) |

---

## Tuning Guide

### Robot is clipping obstacle corners
→ Raise `lethal_cost_threshold` to `253` (INSCRIBED) or increase
`inflation_radius` in your costmap config.

### Planner is too slow
→ Reduce `initial_path_points`, increase `step_size`, or lower
`segment_check_steps`.

### Path has sharp corners
→ Enable `enable_smoothing: true` and raise `smoothing_iterations` to 4–6.

### Fails in narrow corridors
→ Lower `step_size` to `0.02` and raise `max_push_iterations` to `500`.
The perpendicular-push approach inherently struggles here; consider using
a grid planner (Smac/NavFn) as fallback for tight spaces.

### Non-deterministic behavior
→ The fallback is fully deterministic (always pushes perpendicular-left).
If you see inconsistent paths, check that your costmap is stable between
plan requests.

---

## When to Use LCP vs Grid Planners

| Scenario | LCP | NavFn / Smac |
|---|---|---|
| Open environments | ✅ Excellent | ✅ Good |
| Scattered obstacles | ✅ Excellent | ✅ Good |
| Dense / cluttered maps | ⚠️ May struggle | ✅ Better |
| Narrow corridors | ⚠️ Difficult | ✅ Better |
| Dynamic replanning | ✅ Fast startup | ⚠️ Full grid search |
| Path interpretability | ✅ Very clear | ⚠️ Less intuitive |

---

## License

Apache 2.0 — see LICENSE for details.
