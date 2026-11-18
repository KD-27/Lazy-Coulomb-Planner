#!/bin/bash
# Run R2 (Lai et al., github.com/LaiYanKai/2D) on the blocked grids and vertex queries
# that run_anyangle.sh exported, so R2 sees exactly the obstacles and pairs Anya and JPS saw.
# Usage: run_r2.sh <anyangle_dir> <out_dir> [repeats]
# Needs the R2 collection built in Release into $R2_BUILD (default
# ~/lcp_jazzy_ws/baselines/r2/build/Release). Each repeat runs every query once
# (--num_expts); the code times only the search (alg->run) with high_resolution_clock.
set -euo pipefail
aa=$(realpath "$1"); out=$(realpath -m "$2"); repeats=${3:-5}
here=$(dirname "$(realpath "$0")")
exe=${R2_BUILD:-$HOME/lcp_jazzy_ws/baselines/r2/build/Release}/run/run_exe
mkdir -p "$out/maps" "$out/results"
names=()
for q in "$aa"/grids/*.queries.csv; do
  name=$(basename "$q" .queries.csv)
  safe=${name//./p}   # R2 derives file names with replace_extension: no dots allowed
  python3 "$here/export_movingai.py" "$aa/grids/$name.grid.txt.gz" "$q" "$out/maps" "$safe"
  names+=("$safe")
done
for name in "${names[@]}"; do
  ls "$out/results/$name".R2.*.results > /dev/null 2>&1 && continue   # resumable
  # R2 does not return on some unreachable goals: the driver drops a query that hangs
  # for ${R2_TIMEOUT:-20} s and records it in $name.timeouts.txt (counted as no path).
  python3 "$here/run_r2_safe.py" "$exe" "$out/maps" "$out/results" "$name" "$repeats" "${R2_TIMEOUT:-20}"
  echo "$(date +%H:%M:%S) $name done"
done
rmdir "$out/results/_work" 2>/dev/null || true
echo "R2 DONE"
