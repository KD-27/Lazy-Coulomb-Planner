# Lazy Coulomb Planner — Nav2 Global Planner Plugin

A Nav2 global planner plugin for ROS 2 Jazzy that implements reactive path
planning inspired by **electrostatic repulsion (Coulomb's law)**.

**Developed by Kaveesha Dhananjaya**

---

## Algorithm Overview

The planner starts with the laziest possible assumption — a straight line from
start to goal — and only does work when that line is blocked.

```
1. Draw straight line  S ──────────────────── G
2. Find the FIRST segment that touches an obstacle cell
3. Insert a waypoint inside that cell
4. Push it perpendicular to the segment until clear (Coulomb force, below)
5. Lock it (it won't move again)
6. Repeat from step 2
7. Remove redundant waypoints, smooth, resample
```

### Coulomb push

Every obstacle cell within `charge_radius` of the waypoint repels it with a
force of magnitude `1 / r^force_exponent` (r in cells). The forces are summed
and projected onto the normal of the segment the waypoint sits on. The sign of
that projection picks the side the waypoint leaves on; its size sets the step
length, capped at `max_push_step`. With `commit_direction: true` the side
chosen on the first step is kept, which stops the waypoint oscillating between
two obstacles.

### Failure instead of a bad path

The planner never returns a path it knows to be in collision. Segments are
checked with an exact grid traversal, smoothing passes are kept only if they
stay collision free, and if a waypoint cannot be cleared or the correction
budget runs out the planner throws `nav2_core::NoValidPathCouldBeFound`, so a
fallback planner can take over.

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
| `force_model` | `coulomb` | `coulomb`, or `probe` for the legacy nearest-clear-side rule |
| `force_exponent` | 2.0 | n in the `1 / r^n` repulsion law (1.0 or 2.0) |
| `charge_radius` | 1.0 | Obstacle cells within this range repel the waypoint (m) |
| `commit_direction` | true | Keep the side chosen on the first push step |
| `repulsion_strength` | 1.0 | Cells moved per unit of normalised force |
| `max_push_step` | 0.0 | Cap on one push step (m); 0 means one costmap cell |
| `force_balance_threshold` | 0.05 | Below this force the push is a fixed kick |
| `max_iterations` | 500 | Max waypoints inserted before reporting failure |
| `max_push_iterations` | 400 | Max steps to push one waypoint clear |
| `lethal_cost_threshold` | 253.0 | Cost at or above this is an obstacle |
| `simplify_path` | true | Remove waypoints whose neighbours join directly |
| `enable_smoothing` | true | Chaikin smoothing, kept only if collision free |
| `smoothing_iterations` | 3 | Chaikin passes |
| `output_spacing` | 0.1 | Max distance between output poses (m); 0 disables |

---

## Tuning Guide

### Fails in cluttered or indoor maps
→ Raise `charge_radius` (2–4 m). The waypoint then "sees" more of the obstacle
and picks the better side more often, at the cost of planning time, which
grows with the square of the radius.

### Planner is too slow
→ Lower `charge_radius`.

### Path has sharp corners
→ Raise `smoothing_iterations`. Passes that would cut into an obstacle are
dropped automatically.

### Fails in narrow corridors or mazes
→ Expected: a single sideways push cannot route around dead ends. Catch the
failure and fall back to a grid planner (Smac/NavFn).

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
