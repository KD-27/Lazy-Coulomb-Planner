#!/bin/bash
# Replays the two fixed_left requests whose paths touched cost 253 (ablation_side_rule),
# with fixed_left, coulomb and probe, on ROS domain 88 (the driving runs use 87).
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
src=$here/../ablation_side_rule
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=88 ROS_LOG_DIR=$here/roslog
set -u
go() {  # <map> <request>
  setsid ros2 launch "$repo/benchmark/config/bringup_headless.launch.py" params:="$src/planners.yaml" \
    map:="$src/maps/$1.yaml" > "$here/$1.bringup.log" 2>&1 &
  local pid=$!
  for _ in $(seq 1 120); do grep -q "Managed nodes are active" "$here/$1.bringup.log" && break; sleep 1; done
  sleep 5
  for p in fixed_left coulomb probe; do
    python3 "$here/replay.py" "$here/$1_$p.json" "$p" "$2" 2>&1 | grep -v "^\[" | sed "s/^/$1 $p: /"
  done
  kill -INT -- "-$pid" 2>/dev/null; sleep 2; kill -KILL -- "-$pid" 2>/dev/null
}
go density_20 6.575,37.025001,39.775001,45.675001
go density_30 36.625001,1.225,14.975,13.425
