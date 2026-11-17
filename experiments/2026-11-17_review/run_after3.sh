#!/bin/bash
# Waits for AFTER2 DONE, then 5 more MPPI sandbox runs per planner (all four, defaults,
# LCP with path reuse on as in ../2026-11-15_v2.1.0/driving_mppi), interleaved, numbered
# 06-10 so they can be pooled with the 15 November runs 01-05. Logs AFTER3 DONE.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ws/benchmark/simruns/roslog}
log=$here/run.log
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "AFTER2 DONE" "$log"; do sleep 60; done
drive() {  # <planner> <out dir>
  local planner=$1 out=$2 attempt
  [ -f "$out/$planner.csv" ] && return 0
  for attempt in 1 2; do
    rm -rf "$out"; mkdir -p "$out"
    docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -e LCP_SIM_CONTROLLER=mppi \
      -v "$ws":"$ws" lcp-jazzy:latest "$repo/sim/run_sim.sh" "$planner" 15 "$out" > "$out.stdout" 2>&1
    if [ -f "$out/$planner.csv" ]; then
      say "  ${out#$here/}: $(tail -1 "$out.stdout")"
      mv "$out.stdout" "$out/stdout.txt"
      return 0
    fi
    say "  ${out#$here/}: attempt $attempt failed: $(tail -1 "$out.stdout")"
    [ -x "$ws/benchmark/simruns/stop_all.sh" ] && "$ws/benchmark/simruns/stop_all.sh" > /dev/null 2>&1
    sleep 10
  done
}
out=$here/driving_mppi_extra; mkdir -p "$out"
say "driving_mppi_extra: start"
for i in 06 07 08 09 10; do
  for planner in lcp navfn smac2d theta; do
    drive "$planner" "$out/${planner}_$i"
  done
done
python3 "$repo/sim/summarise_runs.py" "$out" > "$out/summary.md" 2>&1
say "driving_mppi_extra: done"
say "AFTER3 DONE"
