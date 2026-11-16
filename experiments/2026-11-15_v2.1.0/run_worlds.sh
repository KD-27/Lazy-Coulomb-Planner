#!/bin/bash
# Driving test on worlds built from generated maps (sim/worlds/density_set.yaml),
# TurtleBot3, Regulated Pure Pursuit, planner v2.1.0. Waits for the other chains
# of this directory to finish, checks the setup with one short run, then drives
# 3 runs x 10 goals per world and planner, one run at a time. Resumable.
# Localisation: wheel odometry from the simulator (see below).
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
log=$here/run.log
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
say() { echo "$(date '+%F %T') $*" >> "$log"; }

worlds=$here/driving_worlds_rpp/worlds
mkdir -p "$worlds"
[ -f "$worlds/worlds.csv" ] || python3 "$repo/sim/make_worlds.py" "$repo/sim/worlds/density_set.yaml" "$worlds" > /dev/null
export LCP_SIM_TIMEOUT=2400 LCP_SIM_LEG_TIMEOUT=150
# Pose from the simulator's odometry, not AMCL: among many identical obstacles AMCL
# can lock onto the wrong place (it did in the first attempt, kept in
# driving_worlds_rpp_amcl_aborted/), and that is not what this test measures.
export LCP_SIM_LOCALIZATION=odometry

drive() {  # <world> <planner> <goals> <out dir>
  local world=$1 planner=$2 goals=$3 out=$4 attempt start
  [ -f "$out/$planner.csv" ] && return 0
  start=$(awk -F, -v w="$world" '$1 == w {print $2, $3}' "$worlds/worlds.csv")
  mkdir -p "$(dirname "$out")"
  for attempt in 1 2; do
    rm -rf "$out"
    LCP_SIM_WORLD=$worlds/$world.sdf LCP_SIM_MAP=$worlds/$world.yaml LCP_SIM_START="$start" \
      "$repo/sim/run_sim.sh" "$planner" "$goals" "$out" > "$out.stdout" 2>&1
    if [ -f "$out/$planner.csv" ]; then
      say "  $world/$(basename "$out"): $(tail -1 "$out.stdout")"
      rm -f "$out.stdout"
      return 0
    fi
    say "  $world/$(basename "$out"): attempt $attempt failed: $(tail -1 "$out.stdout")"
    [ -x "$ws/benchmark/simruns/stop_all.sh" ] && "$ws/benchmark/simruns/stop_all.sh" > /dev/null 2>&1
    sleep 10
  done
  return 1
}

say "driving_worlds_rpp: smoke test"
if ! drive density_10 lcp 2 "$here/driving_worlds_rpp/smoke/lcp"; then
  say "driving_worlds_rpp: smoke test FAILED, stopping"
  say "WORLDS STOPPED"
  exit 1
fi
say "driving_worlds_rpp: start"
for i in 01 02 03; do
  for world in open density_10 density_20 density_30 shape_u corridor_1.2; do
    for planner in lcp navfn smac2d theta; do
      drive "$world" "$planner" 10 "$here/driving_worlds_rpp/$world/${planner}_$i"
    done
  done
done
for world in open density_10 density_20 density_30 shape_u corridor_1.2; do
  python3 "$repo/sim/summarise_runs.py" "$here/driving_worlds_rpp/$world" > "$here/driving_worlds_rpp/$world/summary.md" 2>&1
done
say "driving_worlds_rpp: done"
say "WORLDS DONE"
