#!/bin/bash
# Headless Gazebo + the Nav2 stack with a TurtleBot3, with the global planner
# swapped for the one named. Needs Nav2 Jazzy, nav2_minimal_tb3_sim, ros_gz and
# this workspace built; nav2_bringup is not required.
# usage: run_sim.sh <lcp|navfn|smac2d|theta> <goals> [out_dir]
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
  "$here/config/nav2_sim_params.yaml" > "$params"
grep -q "$plugin" "$params" || { echo "failed to set planner plugin"; exit 1; }

setsid ros2 launch "$here/launch/tb3_sim.launch.py" params_file:="$params" \
  map:="$here/maps/tb3_sandbox.yaml" > "$out/$planner.launch.log" 2>&1 &
launch_pid=$!
# Stop every simulation and navigation process on any exit, including Ctrl-C or a timeout.
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null
  sleep 3
  kill -KILL -- "-$launch_pid" 2>/dev/null
  rm -f "$params"
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
# The robot spawns at (-2.0, -0.5) in the tb3_sandbox world.
# A healthy run takes a few minutes; the limit only catches a hung simulation.
timeout 1200 python3 "$here/drive.py" "$here/maps/tb3_sandbox.yaml" "$out/$planner.csv" "$planner" \
  "$goals" -2.0 -0.5 2>&1 | grep -v "^\[INFO\]"
[ "${PIPESTATUS[0]}" -eq 0 ] || { echo "$planner: run did not complete"; exit 1; }
exit 0
