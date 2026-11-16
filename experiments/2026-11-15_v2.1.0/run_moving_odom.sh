#!/bin/bash
# Driving test with moving obstacles, planner v2.1.0, TurtleBot3, Regulated Pure
# Pursuit. Six cylinders cross the arena at 0.3 m/s; the behaviour tree replans at
# 1 Hz (Nav2's default) or 5 Hz; the planner server's CPU time is recorded.
# Waits for run_worlds.sh, checks the setup with one short run, then drives
# 2 worlds x 2 rates x 4 planners x 3 runs x 10 goals, one run at a time. Resumable.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
ws=$(cd "$repo/../.." && pwd)
log=$here/run.log
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/roslog}
say() { echo "$(date '+%F %T') $*" >> "$log"; }
# Rerun of run_moving.sh with wheel-odometry localisation (no AMCL), as in
# driving_worlds_rpp; same worlds and obstacle lanes as driving_moving_rpp.
export LCP_SIM_LOCALIZATION=odometry

base=$here/driving_moving_rpp_odom
worlds=$here/driving_moving_rpp/worlds
mkdir -p "$worlds"
if [ ! -f "$worlds/worlds.csv" ]; then
  python3 "$repo/sim/make_worlds.py" "$repo/sim/worlds/moving_set.yaml" "$worlds" > /dev/null
  while IFS=, read -r name sx sy _; do
    [ "$name" = world ] && continue
    python3 "$repo/sim/add_moving_obstacles.py" "$worlds/$name.sdf" "$worlds/$name.yaml" \
      "$worlds/${name}_moving" --count 6 --speed 0.3 --start "$sx" "$sy" > /dev/null
  done < "$worlds/worlds.csv"
fi
export LCP_SIM_TIMEOUT=2400 LCP_SIM_LEG_TIMEOUT=150

drive() {  # <world> <hz> <planner> <goals> <out dir>
  local world=$1 hz=$2 planner=$3 goals=$4 out=$5 attempt start
  [ -f "$out/$planner.csv" ] && return 0
  start=$(awk -F, -v w="$world" '$1 == w {print $2, $3}' "$worlds/worlds.csv")
  mkdir -p "$(dirname "$out")"
  for attempt in 1 2; do
    rm -rf "$out"
    LCP_SIM_MOVERS=$worlds/${world}_moving LCP_SIM_MAP=$worlds/$world.yaml \
      LCP_SIM_START="$start" LCP_SIM_REPLAN_HZ=$hz \
      "$repo/sim/run_sim.sh" "$planner" "$goals" "$out" > "$out.stdout" 2>&1
    if [ -f "$out/$planner.csv" ]; then
      say "  moving_odom/$world/${hz}hz/$(basename "$out"): $(tail -1 "$out.stdout") cpu $(cat "$out/planner_cpu_s.txt" 2>/dev/null) s"
      rm -f "$out.stdout"
      return 0
    fi
    say "  moving_odom/$world/${hz}hz/$(basename "$out"): attempt $attempt failed: $(tail -1 "$out.stdout")"
    [ -x "$ws/benchmark/simruns/stop_all.sh" ] && "$ws/benchmark/simruns/stop_all.sh" > /dev/null 2>&1
    sleep 10
  done
  return 1
}

say "driving_moving_rpp_odom: smoke test"
if ! drive open 5 lcp 2 "$base/smoke/lcp"; then
  say "driving_moving_rpp_odom: smoke test FAILED, stopping"
  say "MOVING ODOM STOPPED"
  exit 1
fi
say "driving_moving_rpp_odom: start"
for i in 01 02 03; do
  for world in open density_10; do
    for hz in 1 5; do
      for planner in lcp navfn smac2d theta; do
        drive "$world" "$hz" "$planner" 10 "$base/$world/${hz}hz/${planner}_$i"
      done
    done
  done
done
for world in open density_10; do
  for hz in 1 5; do
    python3 "$repo/sim/summarise_runs.py" "$base/$world/${hz}hz" > "$base/$world/${hz}hz/summary.md" 2>&1
  done
done
say "driving_moving_rpp_odom: done"
say "MOVING ODOM DONE"
