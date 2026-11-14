#!/bin/bash
# Headless Gazebo + full Nav2 with a TurtleBot3, global planner swapped for the
# one named. Run where Nav2 Jazzy, nav2_bringup and this workspace are available
# (for example the image from benchmark/Dockerfile).
# usage: run_sim.sh <lcp|navfn> <goals> [out_dir]
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
# Own DDS domain, so other ROS nodes on this machine cannot interfere.
export ROS_DOMAIN_ID=${LCP_ROS_DOMAIN_ID:-87}
planner=$1; goals=$2; out=${3:-$here/results}
bringup=$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup

case $planner in
  lcp)   plugin="lazy_coulomb_planner::LazyCoulombPlanner" ;;
  navfn) plugin="nav2_navfn_planner::NavfnPlanner" ;;
  *) echo "planner must be lcp or navfn"; exit 2 ;;
esac

# Stock Nav2 parameters with only the global planner plugin replaced.
params=/tmp/nav2_params_$planner.yaml
sed "s|plugin: \"nav2_navfn_planner::NavfnPlanner\"|plugin: \"$plugin\"|" \
  "$bringup/params/nav2_params.yaml" > "$params"
grep -q "$plugin" "$params" || { echo "failed to set planner plugin"; exit 1; }

setsid ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False \
  params_file:="$params" > "$out/$planner.launch.log" 2>&1 &
launch_pid=$!
sleep 25
# The robot spawns at (-2.0, -0.5) in the tb3_sandbox world.
python3 "$here/drive.py" "$bringup/maps/tb3_sandbox.yaml" "$out/$planner.csv" "$planner" \
  "$goals" -2.0 -0.5 2>&1 | grep -v "^\[INFO\]"
kill -INT -- "-$launch_pid" 2>/dev/null
sleep 3
kill -KILL -- "-$launch_pid" 2>/dev/null
exit 0
