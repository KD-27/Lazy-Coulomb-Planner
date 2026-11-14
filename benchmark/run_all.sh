#!/bin/bash
# Runs the benchmark on all six maps inside the lcp-jazzy Docker image.
# Run from the host; the colcon workspace must already be built in the image.
# usage: run_all.sh <tag> <pairs> <planner>...      e.g. run_all.sh final 100 Navfn Smac2d ThetaStar LCP
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)
rel=${here#"$ws"/}
tag=$1; pairs=$2; shift 2
"$here/fetch_nav2_maps.sh" > /dev/null
for map in nav2/100by100_10 nav2/100by100_15 nav2/100by100_20 event_venue maze house; do
  name=$(basename "$map")
  echo "################ $name"
  docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ws":/ws lcp-jazzy:latest \
    "/ws/$rel/bench.sh" "/ws/$rel/config/planners.yaml" "/ws/$rel/maps/$map.yaml" \
    "/ws/$rel/results/${tag}_$name.csv" "$pairs" "$@" 2>&1 | tail -$(( $# + 2 ))
done
