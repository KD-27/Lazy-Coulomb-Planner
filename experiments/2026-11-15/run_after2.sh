#!/bin/bash
# Third unattended chain: after run_after.sh, repeat three ablations with the grid
# fallback off, so they show what the lazy search solves on its own.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
log=$here/overnight.log
export ROS_LOG_DIR=$ws/benchmark/simruns/roslog
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "SECOND CHAIN DONE" "$log" 2>/dev/null; do sleep 60; done
for name in force_law_lazy charge_radius_lazy waypoint_margin_lazy; do
  out=$here/ablation_$name
  mkdir -p "$out"
  say "ablation $name: start"
  python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/$name.yaml" "$out" \
    >> "$out/progress.log" 2>&1
  say "ablation $name: done"
done
say "THIRD CHAIN DONE"
