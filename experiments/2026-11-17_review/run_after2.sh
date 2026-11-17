#!/bin/bash
# Waits for AFTER DONE, then resumes tuned_baselines (it stopped after 9 of 21 maps on a
# summary bug, fixed in run_suite.py; finished maps are skipped). Logs AFTER2 DONE.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ws/benchmark/simruns/roslog}
log=$here/run.log
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "AFTER DONE" "$log"; do sleep 60; done
out=$here/tuned_baselines
say "tuned_baselines (resume): start"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/tuned_baselines.yaml" "$out" >> "$out/progress.log" 2>&1
python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
python3 "$repo/benchmark/report_suite.py" "$out" "Tuned grid baselines, planner v2.1.0" > /dev/null 2>&1
say "tuned_baselines (resume): done"
say "AFTER2 DONE"
