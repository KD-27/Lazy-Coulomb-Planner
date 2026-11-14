# Driving test

Drives a simulated TurtleBot3 through a fixed sequence of goals in headless
Gazebo with the full Nav2 stack, once per global planner, and records how each
leg went. It checks that planned paths can actually be driven, which the
planner-only benchmark in `benchmark/` cannot show.

## Running

```bash
colcon build --packages-select lazy_coulomb_planner      # from the workspace root
sim/run_sim.sh lcp 15       # 15 goals with the Lazy Coulomb Planner
sim/run_sim.sh navfn 15     # the same goals with NavFn
sim/run_sim.sh smac2d 15    # ... or Smac 2D, or theta for Theta*
```

Needs Nav2 Jazzy (controller, planner, behaviours, BT navigator, AMCL, map
server, smoother, velocity smoother, collision monitor, Regulated Pure Pursuit),
`nav2_minimal_tb3_sim` and `ros_gz`. `nav2_bringup` is not required.

Each run uses its own ROS domain and Gazebo partition, so it does not interfere
with other ROS nodes on the machine. To run several at once, give each a
different `LCP_ROS_DOMAIN_ID` and output directory:

```bash
LCP_ROS_DOMAIN_ID=91 sim/run_sim.sh lcp 15 /tmp/run_a &
LCP_ROS_DOMAIN_ID=81 sim/run_sim.sh navfn 15 /tmp/run_b &
```

## Files

| File | Purpose |
|---|---|
| `run_sim.sh` | Starts the simulation with the chosen planner, runs the goals, stops everything |
| `launch/tb3_sim.launch.py` | Gazebo, robot, localisation and navigation servers |
| `config/nav2_sim_params.yaml` | Nav2 parameters, derived from `nav2_bringup` with Regulated Pure Pursuit as controller |
| `maps/tb3_sandbox.*` | Map of the sandbox world, from `nav2_bringup` (Apache-2.0) |
| `drive.py` | Samples the goals (fixed seed), sends them one by one, writes the CSV |
| `results/` | CSVs of the last run for each planner |

The controller is Regulated Pure Pursuit because it follows the global path
closely and stops when the path ahead is too close to an obstacle, so the result
depends on the quality of the planned path. Results vary from run to run; compare
planners over several runs, not one.

Next to each CSV the run writes a trace, `<planner>.trace.jsonl`: the robot's
pose five times a second, every path the planner publishes, the velocity
commands and the controller's warnings, each with a wall-clock time. It shows
what the robot and the path looked like at the moment of a stop.

The script refuses to start if other nodes are already running on its ROS
domain, and gives up if the simulator or localisation fails to start.

## CSV columns

`planned_m` is the length of the path planned before the leg starts,
`sim_time_s` and `travelled_m` are measured while driving, `min_scan_range_m` is
the closest lidar return during the leg, and `recoveries` counts the recovery
behaviours Nav2 triggered.
