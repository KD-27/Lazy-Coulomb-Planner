#!/bin/bash
# Runs the benchmark on all six maps. Needs Nav2 Jazzy installed and this
# workspace built (natively, or run this script inside the Docker image).
# usage: run_all.sh <tag> <pairs> <planner>...      e.g. run_all.sh final 100 Navfn Smac2d ThetaStar LCP
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)
tag=$1; pairs=$2; shift 2
"$here/fetch_nav2_maps.sh" > /dev/null
for map in nav2/100by100_10 nav2/100by100_15 nav2/100by100_20 event_venue maze house; do
  name=$(basename "$map")
  echo "################ $name"
  "$here/bench.sh" "$here/config/planners.yaml" "$here/maps/$map.yaml" \
    "$here/results/${tag}_$name.csv" "$pairs" "$@" 2>&1 | tail -$(( $# + 2 ))
done
