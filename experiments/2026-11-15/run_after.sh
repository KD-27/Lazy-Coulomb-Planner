#!/bin/bash
# Second unattended chain: starts when run_overnight.sh has finished, then repeats
# the controlled benchmark with other random seeds. One job at a time.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
log=$here/overnight.log
export ROS_LOG_DIR=$ws/benchmark/simruns/roslog
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "ALL DONE" "$log" 2>/dev/null; do sleep 60; done
out=$here/controlled_seeds_v2.0.0
mkdir -p "$out"
say "controlled suite, seeds 2 to 5: start"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/controlled_seeds.yaml" "$out" \
  >> "$out/progress.log" 2>&1
python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
say "controlled suite, seeds 2 to 5: done"
say "SECOND CHAIN DONE"
