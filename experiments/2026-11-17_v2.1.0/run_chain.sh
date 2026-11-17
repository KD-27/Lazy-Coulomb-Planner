#!/bin/bash
# 2026-11-17, planner v2.1.0, one job at a time, resumable (finished maps are skipped):
#  1. Java any-angle / grid baselines (Anya, JPS, Theta*, Lazy Theta*) on the maps and
#     pairs of ../2026-11-15_v2.1.0/controlled
#  2. ablation_line_then_astar: LCP against "straight line, else A*" and a fixed push side
#  3. ablation_clearance_stages: clearance after each post-processing stage
# Progress goes to run.log, which ends with CHAIN DONE.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
log=$here/run.log
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
say() { echo "$(date '+%F %T') $*" >> "$log"; }

suite() {  # <suite file> <output directory name> <title>
  local out=$here/$2
  mkdir -p "$out"
  say "$2: start"
  python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/$1" "$out" >> "$out/progress.log" 2>&1
  python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
  python3 "$repo/benchmark/report_suite.py" "$out" "$3" > /dev/null 2>&1
  say "$2: done"
}

say "planner: $(git -C "$repo" describe --tags --dirty)"
say "anyangle_controlled: start"
"$repo/benchmark/anyangle/run_anyangle.sh" "$here/../2026-11-15_v2.1.0/controlled" \
  "$here/anyangle_controlled" 5 >> "$here/anyangle_controlled.log" 2>&1
say "anyangle_controlled: $(tail -1 "$here/anyangle_controlled.log")"
suite ablation/line_then_astar.yaml ablation_line_then_astar "Ablation: straight line, else A*, planner v2.1.0"
suite ablation/clearance_stages.yaml ablation_clearance_stages "Ablation: clearance by stage, planner v2.1.0"
say "CHAIN DONE"
