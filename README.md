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
7. If a waypoint cannot be placed, back up and try the other side
8. Remove redundant waypoints, smooth, resample
```

### Coulomb push

Every obstacle cell within `charge_radius` of the waypoint repels it with a
force of magnitude `1 / r^force_exponent` (r in cells). The forces are summed
and projected onto the normal of the segment the waypoint sits on. The sign of
that projection picks the side the waypoint leaves on; its size sets the step
length, capped at `max_push_step`. With `commit_direction: true` the side
chosen on the first step is kept, which stops the waypoint oscillating between
two obstacles.

### Retrying the other side

Each waypoint could have left its obstacle on either side. The side not taken
is remembered together with the path as it was. When a branch dead-ends (a
waypoint cannot be cleared, makes no progress, or the detour exceeds
`max_path_stretch` times the straight-line distance) the planner resumes from
the remembered alternative with the shortest partial path. `max_iterations`
bounds the total number of placement attempts, so it is the knob that trades
success in cluttered maps against the time spent before giving up.

### Collision-safe smoothing

Chaikin corner cutting replaces each corner P with a point Q on the incoming
segment and a point R on the outgoing one. Q and R lie on segments that are
already clear, so the only new piece is the chord Q -> R. The chord is checked
against the costmap; if it touches an obstacle the cut is retried closer to
the corner (1/4, 1/8, 1/16 of the segment) and if none fits the corner is
kept. The smoothed path is therefore collision free by construction.

### Failure instead of a bad path

The planner never returns a path it knows to be in collision. Segments are
checked with an exact grid traversal, the finished path gets a final collision
check, and if no side choice works or the correction
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
| `start_escape_radius` | 0.5 | If the robot sits on an occupied cell, plan from the nearest free cell within this range (m); 0 disables |
| `waypoint_margin` | 0.3 | Extra push past the obstacle edge when free (m); gives corners room to be smoothed |
| `retry_other_side` | true | Back up and try the other side when a branch dead-ends |
| `retry_order` | `shortest` | Retry from the `shortest` partial path, or the most `recent` decision |
| `max_path_stretch` | 5.0 | Abandon detours longer than this × the straight-line distance |
| `max_iterations` | 2000 | Max waypoint placement attempts before reporting failure |
| `max_push_iterations` | 400 | Max steps to push one waypoint clear |
| `lethal_cost_threshold` | 253.0 | Cost at or above this is an obstacle |
| `simplify_path` | true | Remove waypoints whose neighbours join directly |
| `enable_smoothing` | true | Collision-safe Chaikin smoothing |
| `smoothing_iterations` | 5 | Chaikin passes; each roughly halves the corner angles |
| `output_spacing` | 0.1 | Max distance between output poses (m); 0 disables |

---

## Tuning Guide

### Fails in cluttered or indoor maps
→ Raise `max_iterations` so more side combinations are tried, or raise `charge_radius` (2–4 m). The waypoint then "sees" more of the obstacle
and picks the better side more often, at the cost of planning time, which
grows with the square of the radius.

### Planner is too slow, or takes too long to report failure
→ Lower `max_iterations` (time to give up) or `charge_radius` (time per push).

### Path has sharp corners
→ Raise `waypoint_margin` so waypoints sit further from the obstacle edge, and
`smoothing_iterations`. A corner stays sharp only where every cut would touch
an obstacle.

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
