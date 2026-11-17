#!/bin/bash
# Waits for CHAIN DONE in run.log, then runs the side-rule ablation (8 maps). Logs AFTER DONE.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ws/benchmark/simruns/roslog}
log=$here/run.log
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "CHAIN DONE" "$log"; do sleep 60; done
out=$here/ablation_side_rule; mkdir -p "$out"
say "ablation_side_rule: start"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/side_rule.yaml" "$out" >> "$out/progress.log" 2>&1
python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
python3 "$repo/benchmark/report_suite.py" "$out" "Ablation: side rule in one run, planner v2.1.0" > /dev/null 2>&1
say "ablation_side_rule: done"
say "AFTER DONE"
