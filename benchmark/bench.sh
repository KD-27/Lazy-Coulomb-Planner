#!/bin/bash
# Benchmarks planners on one map. Run inside an environment with Nav2 Jazzy and
# this workspace built (for example the image from benchmark/Dockerfile).
# usage: bench.sh <params.yaml> <map.yaml> <out.csv> <pairs> <planner>...
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
# LCP_BENCH_SETUP: benchmark the planner of another workspace (for comparing versions).
source "${LCP_BENCH_SETUP:-$ws/install/setup.bash}"
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
# Own DDS domain, so other ROS nodes on this machine cannot interfere.
export ROS_DOMAIN_ID=${LCP_ROS_DOMAIN_ID:-87}
set -u  # after sourcing: the ROS setup files read unset variables
params=$1; map=$2; out=$3; pairs=$4; shift 4

# Own process group, so every node can be stopped afterwards: SIGINT to
# `ros2 launch` alone does not reliably reach its children.
setsid ros2 launch "$here/config/bringup_headless.launch.py" params:="$params" map:="$map" \
  > "${out%.csv}.bringup.log" 2>&1 &
launch_pid=$!
# Large maps take a while to load and inflate: wait for the servers, up to 3 minutes.
for _ in $(seq 1 180); do
  grep -q "Managed nodes are active" "${out%.csv}.bringup.log" 2>/dev/null && break
  sleep 1
done
sleep 5
python3 "$here/lcp_bench.py" "$out" "$pairs" "$@" 2>&1 | grep -v "^\["
kill -INT -- "-$launch_pid" 2>/dev/null
sleep 2
kill -KILL -- "-$launch_pid" 2>/dev/null
exit 0
