#!/bin/bash
# The full measurement programme on planner v2.1.0, unattended, one job at a time:
# controlled benchmark, its repeats over seeds, the ablations, and the driving
# test with two controllers. Every stage is resumable: finished pieces are
# skipped, so the script can simply be started again. Progress goes to run.log.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
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

# -- 1. controlled benchmark and its repeats over seeds -------------------------
suite controlled.yaml controlled "Controlled benchmark, planner v2.1.0"
suite controlled_seeds.yaml controlled_seeds "Controlled benchmark, seeds 2 to 5, planner v2.1.0"
python3 "$repo/benchmark/aggregate_seeds.py" "$here/controlled_seeds/all_seeds" \
  "$here/controlled" "$here/controlled_seeds" > /dev/null 2>&1
say "benchmark: done"

# -- 2. ablations ------------------------------------------------------------------
for name in force_law force_law_lazy charge_radius charge_radius_lazy retry waypoint_margin \
  waypoint_margin_lazy comfort_cost fallback_budget grid_search_weight smoothing; do
  suite "ablation/$name.yaml" "ablation_$name" "Ablation: $name, planner v2.1.0"
done
say "ablations: done"

# -- 3. driving ----------------------------------------------------------------------
drive() {  # <planner> <out dir> [docker] : one run, repeated once if it does not complete
  local planner=$1 out=$2 attempt
  [ -f "$out/$planner.csv" ] && return 0
  for attempt in 1 2; do
    rm -rf "$out"
    if [ "${3:-}" = docker ]; then
      mkdir -p "$out"
      docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -e LCP_SIM_CONTROLLER=mppi \
        -v "$ws":"$ws" lcp-jazzy:latest "$repo/sim/run_sim.sh" "$planner" 15 "$out" > "$out.stdout" 2>&1
    else
      "$repo/sim/run_sim.sh" "$planner" 15 "$out" > "$out.stdout" 2>&1
    fi
    if [ -f "$out/$planner.csv" ]; then
      say "  $(basename "$out"): $(tail -1 "$out.stdout")"
      rm -f "$out.stdout"
      return 0
    fi
    say "  $(basename "$out"): attempt $attempt failed: $(tail -1 "$out.stdout")"
    # Local helper that clears stray simulation processes, if this machine has it.
    [ -x "$ws/benchmark/simruns/stop_all.sh" ] && "$ws/benchmark/simruns/stop_all.sh" > /dev/null 2>&1
    sleep 10
  done
  return 1
}

mkdir -p "$here/driving_rpp" "$here/driving_mppi"
say "driving_rpp: start"
for i in 01 02 03 04 05 06 07 08 09 10; do
  for planner in lcp navfn smac2d theta; do
    drive "$planner" "$here/driving_rpp/${planner}_$i"
  done
done
python3 "$repo/sim/summarise_runs.py" "$here/driving_rpp" > "$here/driving_rpp/summary.md" 2>&1
say "driving_rpp: done"

# The MPPI controller is not installed natively here; it runs in the Docker image.
say "driving_mppi: start"
for i in 01 02 03 04 05; do
  for planner in lcp navfn smac2d theta; do
    drive "$planner" "$here/driving_mppi/${planner}_$i" docker
  done
done
python3 "$repo/sim/summarise_runs.py" "$here/driving_mppi" > "$here/driving_mppi/summary.md" 2>&1
say "driving_mppi: done"
say "ALL DONE"
