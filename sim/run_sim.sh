#!/bin/bash
# Headless Gazebo + the Nav2 stack with a TurtleBot3, with the global planner
# swapped for the one named. Needs Nav2 Jazzy, nav2_minimal_tb3_sim, ros_gz and
# this workspace built; nav2_bringup is not required.
# usage: run_sim.sh <lcp|navfn|smac2d|theta> <goals> [out_dir]
# LCP_SIM_CONTROLLER=mppi drives with the MPPI controller (needs
# nav2_mppi_controller) instead of Regulated Pure Pursuit.
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
# Own DDS domain, so other ROS nodes on this machine cannot interfere.
export ROS_DOMAIN_ID=${LCP_ROS_DOMAIN_ID:-87}
# Gazebo has its own discovery: give each run its own partition as well, so
# several simulations on one network cannot see each other.
export GZ_PARTITION=lcp_sim_${ROS_DOMAIN_ID}
planner=$1; goals=$2; out=${3:-$here/results}
# Another world: LCP_SIM_WORLD (SDF), LCP_SIM_MAP (its map YAML) and
# LCP_SIM_START ("x y", where the robot is spawned). Default: Nav2's sandbox.
world=${LCP_SIM_WORLD:-}
map=${LCP_SIM_MAP:-$here/maps/tb3_sandbox.yaml}
read -r start_x start_y <<< "${LCP_SIM_START:--2.0 -0.5}"
if [ -n "$world" ] && [ -z "${LCP_SIM_MAP:-}" ]; then
  echo "LCP_SIM_WORLD needs LCP_SIM_MAP and LCP_SIM_START as well"; exit 2
fi
export LCP_SIM_WORLD=$world
# LCP_SIM_MOVERS: prefix of a world with moving obstacles (add_moving_obstacles.py);
# its .sdf replaces LCP_SIM_WORLD. LCP_SIM_REPLAN_HZ: how often the behaviour tree
# replans while driving (Nav2's default is 1 Hz).
if [ -n "${LCP_SIM_MOVERS:-}" ]; then
  export LCP_SIM_WORLD=$LCP_SIM_MOVERS.sdf LCP_SIM_MOVERS
fi
case $planner in
  lcp)   plugin="lazy_coulomb_planner::LazyCoulombPlanner" ;;
  navfn) plugin="nav2_navfn_planner::NavfnPlanner" ;;
  smac2d) plugin="nav2_smac_planner::SmacPlanner2D" ;;
  theta) plugin="nav2_theta_star_planner::ThetaStarPlanner" ;;
  *) echo "planner must be lcp, navfn, smac2d or theta"; exit 2 ;;
esac

# Anything already running on this ROS domain (a previous run that was not shut
# down, a benchmark, a stray transform publisher) would corrupt the test.
# Nodes of a run that has just ended take a few seconds to disappear, so wait a little.
for _ in 1 2 3 4 5 6; do
  others=$(ros2 node list --no-daemon 2>/dev/null | grep -v "^/_ros2cli")
  [ -z "$others" ] && break
  sleep 5
done
if [ -n "$others" ]; then
  echo "$planner: ROS domain $ROS_DOMAIN_ID is not empty, refusing to start. Nodes found:"
  echo "$others"
  exit 1
fi

# The test parameters with only the global planner plugin replaced.
mkdir -p "$out"
params=$(mktemp --suffix=.yaml)
sed "s|plugin: .*nav2_navfn_planner::NavfnPlanner.*|plugin: \"$plugin\"|" \
  "$here/config/nav2_sim_params${LCP_SIM_CONTROLLER:+_$LCP_SIM_CONTROLLER}.yaml" > "$params"
grep -q "$plugin" "$params" || { echo "failed to set planner plugin"; exit 1; }
if [ -n "${LCP_SIM_REPLAN_HZ:-}" ]; then
  bt=$(mktemp --suffix=.xml)
  sed "s|<RateController hz=\"1.0\">|<RateController hz=\"$LCP_SIM_REPLAN_HZ\">|" \
    "$(ros2 pkg prefix nav2_bt_navigator)/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml" > "$bt"
  grep -q "hz=\"$LCP_SIM_REPLAN_HZ\"" "$bt" || { echo "failed to set the replanning rate"; exit 1; }
  python3 - "$params" "$bt" <<'PY'
import sys, yaml
params, bt = sys.argv[1], sys.argv[2]
d = yaml.safe_load(open(params))
d['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt
yaml.safe_dump(d, open(params, 'w'), sort_keys=False)
PY
fi
# LCP_SIM_PLANNER_PARAMS: a YAML mapping merged into the global planner's parameters,
# e.g. "{path_reuse_distance: 0.0}" to drive LCP without path reuse.
if [ -n "${LCP_SIM_PLANNER_PARAMS:-}" ]; then
  python3 - "$params" "$LCP_SIM_PLANNER_PARAMS" <<'PY' || { echo "failed to set planner parameters"; exit 1; }
import sys, yaml
params, extra = sys.argv[1], yaml.safe_load(sys.argv[2])
assert isinstance(extra, dict) and extra
d = yaml.safe_load(open(params))
d['planner_server']['ros__parameters']['GridBased'].update(extra)
yaml.safe_dump(d, open(params, 'w'), sort_keys=False)
PY
  echo "planner parameters: $LCP_SIM_PLANNER_PARAMS"
fi

setsid ros2 launch "$here/launch/tb3_sim.launch.py" params_file:="$params" \
  map:="$map" x_pose:="$start_x" y_pose:="$start_y" > "$out/$planner.launch.log" 2>&1 &
launch_pid=$!
# Stop every simulation and navigation process on any exit, including Ctrl-C or a timeout.
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null
  sleep 3
  kill -KILL -- "-$launch_pid" 2>/dev/null
  rm -f "$params" "${bt:-}"
}
trap cleanup EXIT
trap 'exit 1' INT TERM
# Localisation must come up; if it has not within 90 s the start-up has failed.
for _ in $(seq 1 18); do
  grep -q "lifecycle_manager_localization.*Managed nodes are active" "$out/$planner.launch.log" && break
  sleep 5
done
if ! grep -q "lifecycle_manager_localization.*Managed nodes are active" "$out/$planner.launch.log"; then
  echo "$planner: localisation did not start, see $out/$planner.launch.log"
  exit 1
fi
sleep 10
if grep -q "gz-.*process has died" "$out/$planner.launch.log"; then
  echo "$planner: the simulator did not start, see $out/$planner.launch.log"
  exit 1
fi
# Navigation must come up soon after the driver sets the initial pose. If it has
# not after 3 minutes the start-up has failed: stop the driver so the run ends.
( sleep 180
  grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$out/$planner.launch.log" ||
    pkill -KILL -f "sim/drive.py .*$out/$planner.csv" ) &
watchdog=$!
# A healthy run takes a few minutes; the limit only catches a hung simulation.
# -k: the driver does not always exit on SIGTERM, so follow up with SIGKILL.
# CPU time used by the planner server while the robot drives (user + system, s).
cpu_of_planner() {
  local pid
  pid=$(pgrep -f "nav2_planner/planner_server" | head -1)
  [ -n "$pid" ] && awk '{print ($14 + $15) / 100}' "/proc/$pid/stat" 2>/dev/null
}
cpu_before=$(cpu_of_planner)
timeout -k 10 "${LCP_SIM_TIMEOUT:-1200}" python3 "$here/drive.py" "$map" "$out/$planner.csv" \
  "$planner" "$goals" "$start_x" "$start_y" 2>&1 | grep -v "^\[INFO\]"
status=${PIPESTATUS[0]}
cpu_after=$(cpu_of_planner)
if [ -n "$cpu_before" ] && [ -n "$cpu_after" ]; then
  awk -v a="$cpu_before" -v b="$cpu_after" 'BEGIN {printf "%.2f\n", b - a}' > "$out/planner_cpu_s.txt"
fi
kill "$watchdog" 2>/dev/null
[ "$status" -eq 0 ] || { echo "$planner: run did not complete"; exit 1; }
exit 0
