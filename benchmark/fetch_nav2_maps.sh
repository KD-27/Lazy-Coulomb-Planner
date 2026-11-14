#!/bin/bash
# Downloads the three 100 m x 100 m random-obstacle maps (10 / 15 / 20 % occupied)
# that ship with Nav2's tools/planner_benchmarking into maps/nav2/.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
base=https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/tools/planner_benchmarking
mkdir -p "$here/maps/nav2"
for n in 100by100_10 100by100_15 100by100_20; do
  for ext in pgm yaml; do
    [ -f "$here/maps/nav2/$n.$ext" ] || curl -fsSL "$base/$n.$ext" -o "$here/maps/nav2/$n.$ext"
  done
done
echo "maps in $here/maps/nav2"
