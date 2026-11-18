#!/bin/bash
# Second review, item 2: Coulomb vs probe, fallback on/off, 21 controlled maps. Logs to run.log.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ws/benchmark/simruns/roslog}
log=$here/run.log
say() { echo "$(date '+%F %T') $*" >> "$log"; }
out=$here/side_rule_controlled; mkdir -p "$out"
say "side_rule_controlled: start"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/side_rule_controlled.yaml" "$out" >> "$out/progress.log" 2>&1
say "run_suite exit $?"
python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
python3 "$repo/benchmark/report_suite.py" "$out" "Side rule on the 21 controlled maps, planner v2.1.0" > /dev/null 2>&1
say "side_rule_controlled: done"
