#!/bin/bash
# After run_all.sh: rerun the comfort-cost ablation, which crashed because a
# planner named `off` was read by YAML as a boolean.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
log=$here/run.log
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "ALL DONE" "$log" 2>/dev/null; do sleep 60; done
out=$here/ablation_comfort_cost
mkdir -p "$out"
say "ablation_comfort_cost (rerun): start"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/comfort_cost.yaml" "$out" >> "$out/progress.log" 2>&1
python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
python3 "$repo/benchmark/report_suite.py" "$out" "Ablation: comfort_cost, planner v2.1.0" > /dev/null 2>&1
rm -rf "$here/.ablation_comfort_cost_crashed"
say "ablation_comfort_cost (rerun): done"
say "RERUN DONE"
