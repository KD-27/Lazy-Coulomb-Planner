#!/bin/bash
# Run the Java any-angle / grid baselines on the maps and pairs of a finished
# benchmark suite directory (e.g. experiments/2026-11-15_v2.1.0/controlled).
# Usage: run_anyangle.sh <suite_dir> <out_dir> [repeats]
# Needs the Any-Angle-Pathfinding collection compiled with AnyAngleBench.java into
# $AAP_BUILD (default ~/lcp_jazzy_ws/baselines/aap_build).
set -euo pipefail
suite=$(realpath "$1"); out=$(realpath -m "$2"); repeats=${3:-5}
here=$(dirname "$(realpath "$0")")
build=${AAP_BUILD:-$HOME/lcp_jazzy_ws/baselines/aap_build}
algos="Anya16 JPS JPS_PS Theta LazyTheta"
mkdir -p "$out/grids"
for csv in "$suite"/*.csv; do
  name=$(basename "$csv" .csv)
  [ -f "$suite/maps/$name.yaml" ] || continue
  [ -f "$out/$name.csv" ] && continue            # resumable
  python3 "$here/export_grid.py" "$suite/maps/$name.yaml" "$csv" "$out/grids/$name"
  java -Xmx6g -cp "$build" AnyAngleBench "$out/grids/$name.grid.txt" \
       "$out/grids/$name.queries.csv" "$out/$name.csv.part" "$repeats" $algos
  mv "$out/$name.csv.part" "$out/$name.csv"
  echo "$(date +%H:%M:%S) $name done"
done
gzip -f "$out"/grids/*.grid.txt
echo "ANYANGLE DONE"
