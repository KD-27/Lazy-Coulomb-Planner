#!/bin/bash
# 2026-11-17, planner v2.1.0, experiments requested by the external review. One job at a
# time, resumable (finished maps and runs are skipped). Progress in run.log (ends CHAIN DONE).
#  1. tuned_baselines: Nav2 grid planners with their cost terms off (NavFn with A*),
#     against LCP and "line, else weighted A*", 21 controlled maps
#  2. driving_reuse_mppi: LCP with path reuse off vs on, MPPI, sandbox, 5 + 5 runs interleaved
#  3. driving_reuse_rpp:  the same with Regulated Pure Pursuit, 10 + 10 runs
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
log=$here/run.log
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ws/benchmark/simruns/roslog}
say() { echo "$(date '+%F %T') $*" >> "$log"; }

say "planner: $(git -C "$repo" describe --tags --dirty)"
out=$here/tuned_baselines; mkdir -p "$out"
say "tuned_baselines: start"
python3 "$repo/benchmark/run_suite.py" "$repo/benchmark/suites/ablation/tuned_baselines.yaml" "$out" >> "$out/progress.log" 2>&1
python3 "$repo/benchmark/routability.py" "$out" > "$out/routability.txt" 2>&1
python3 "$repo/benchmark/report_suite.py" "$out" "Tuned grid baselines, planner v2.1.0" > /dev/null 2>&1
say "tuned_baselines: done"

drive() {  # <out dir> <reuse on|off> [docker] : one LCP run, repeated once if it does not complete
  local out=$1 extra="" attempt
  [ "$2" = off ] && extra="{path_reuse_distance: 0.0}"
  [ -f "$out/lcp.csv" ] && return 0
  for attempt in 1 2; do
    rm -rf "$out"; mkdir -p "$out"
    if [ "${3:-}" = docker ]; then
      docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -e LCP_SIM_CONTROLLER=mppi \
        -e LCP_SIM_PLANNER_PARAMS="$extra" -v "$ws":"$ws" lcp-jazzy:latest \
        "$repo/sim/run_sim.sh" lcp 15 "$out" > "$out.stdout" 2>&1
    else
      LCP_SIM_PLANNER_PARAMS="$extra" "$repo/sim/run_sim.sh" lcp 15 "$out" > "$out.stdout" 2>&1
    fi
    if [ -f "$out/lcp.csv" ]; then
      say "  ${out#$here/}: $(grep -c . "$out.stdout") lines, $(grep 'planner parameters' "$out.stdout") | $(tail -1 "$out.stdout")"
      mv "$out.stdout" "$out/stdout.txt"
      return 0
    fi
    say "  ${out#$here/}: attempt $attempt failed: $(tail -1 "$out.stdout")"
    [ -x "$ws/benchmark/simruns/stop_all.sh" ] && "$ws/benchmark/simruns/stop_all.sh" > /dev/null 2>&1
    sleep 10
  done
  return 1
}
batch() {  # <name> <runs> [docker]
  say "$1: start"
  for i in $(seq -f %02g 1 "$2"); do
    drive "$here/$1/reuse_off/lcp_$i" off "${3:-}"
    drive "$here/$1/reuse_on/lcp_$i" on "${3:-}"
  done
  for r in reuse_off reuse_on; do
    python3 "$repo/sim/summarise_runs.py" "$here/$1/$r" > "$here/$1/$r/summary.md" 2>&1
  done
  say "$1: done"
}
batch driving_reuse_mppi 5 docker
batch driving_reuse_rpp 10
say "CHAIN DONE"
