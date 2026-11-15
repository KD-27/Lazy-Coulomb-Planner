#!/bin/bash
# The unattended data collection of the night of 15 November 2026, on planner v2.0.0.
# Strictly one job at a time. Every stage is resumable: finished pieces are skipped.
# Progress goes to overnight.log in this directory.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
log=$here/overnight.log
export ROS_LOG_DIR=$ws/benchmark/simruns/roslog
say() { echo "$(date '+%F %T') $*" >> "$log"; }

# -- 1. controlled benchmark (a first pass may already be running) -------------
while pgrep -f "benchmark/run_suite.py" > /dev/null; do sleep 20; done
say "controlled suite: start (second pass fills any gaps)"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/controlled.yaml" \
  "$here/controlled_v2.0.0" >> "$here/controlled_v2.0.0/progress.log" 2>&1
say "controlled suite: done"

# -- 2. ablations ---------------------------------------------------------------
for name in force_law charge_radius retry waypoint_margin comfort_cost fallback_budget smoothing; do
  out=$here/ablation_$name
  mkdir -p "$out"
  say "ablation $name: start"
  python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/$name.yaml" "$out" \
    >> "$out/progress.log" 2>&1
  say "ablation $name: done"
done

# -- 3. driving, Regulated Pure Pursuit, native ---------------------------------
drive() {  # <planner> <out dir> : one run, retried once if it does not complete
  local planner=$1 out=$2
  [ -f "$out/$planner.csv" ] && return 0
  for attempt in 1 2; do
    rm -rf "$out"
    "$repo/sim/run_sim.sh" "$planner" 15 "$out" > "$out.stdout" 2>&1
    if [ -f "$out/$planner.csv" ]; then
      say "  $(basename "$out"): $(tail -1 "$out.stdout")"
      rm -f "$out.stdout"
      return 0
    fi
    say "  $(basename "$out"): attempt $attempt failed: $(tail -1 "$out.stdout")"
    "$ws/benchmark/simruns/stop_all.sh" > /dev/null 2>&1
    sleep 10
  done
  return 1
}
mkdir -p "$here/driving_rpp"
say "driving (Regulated Pure Pursuit): start"
for i in 01 02 03 04 05 06 07 08 09 10; do
  for planner in lcp navfn smac2d theta; do
    drive "$planner" "$here/driving_rpp/${planner}_$i"
  done
done
say "driving (Regulated Pure Pursuit): done"

# -- 4. driving, MPPI, inside the Docker image ----------------------------------
mkdir -p "$here/driving_mppi"
say "driving (MPPI, Docker): start"
for i in 01 02 03 04 05; do
  for planner in lcp navfn smac2d theta; do
    out=$here/driving_mppi/${planner}_$i
    [ -f "$out/$planner.csv" ] && continue
    for attempt in 1 2; do
      rm -rf "$out"; mkdir -p "$out"
      docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -e LCP_SIM_CONTROLLER=mppi \
        -v "$ws":"$ws" lcp-jazzy:latest "$repo/sim/run_sim.sh" "$planner" 15 "$out" > "$out.stdout" 2>&1
      if [ -f "$out/$planner.csv" ]; then
        say "  $(basename "$out"): $(tail -1 "$out.stdout")"; rm -f "$out.stdout"; break
      fi
      say "  $(basename "$out"): attempt $attempt failed: $(tail -1 "$out.stdout")"
      sleep 10
    done
  done
done
say "driving (MPPI, Docker): done"
say "ALL DONE"
