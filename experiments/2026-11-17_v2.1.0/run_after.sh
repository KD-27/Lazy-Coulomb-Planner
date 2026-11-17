#!/bin/bash
# Waits for CHAIN DONE, then reruns the any-angle baselines with the blocked radius
# fixed to 0.224 m (first run, kept as anyangle_controlled_r0.22, used 0.22 m and so
# saw a slightly more open map than LCP's costmap). Logs AFTER DONE.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
log=$here/run.log
say() { echo "$(date '+%F %T') $*" >> "$log"; }
until grep -q "CHAIN DONE" "$log"; do sleep 30; done
[ -d "$here/anyangle_controlled" ] && [ ! -d "$here/anyangle_controlled_r0.22" ] && \
  mv "$here/anyangle_controlled" "$here/anyangle_controlled_r0.22" && \
  mv "$here/anyangle_controlled.log" "$here/anyangle_controlled_r0.22.log"
say "anyangle_controlled (radius 0.224): start"
"$repo/benchmark/anyangle/run_anyangle.sh" "$here/../2026-11-15_v2.1.0/controlled" \
  "$here/anyangle_controlled" 5 >> "$here/anyangle_controlled.log" 2>&1
python3 "$repo/benchmark/anyangle/compare.py" "$here/../2026-11-15_v2.1.0/controlled" \
  "$here/anyangle_controlled" > /dev/null 2>&1
say "anyangle_controlled (radius 0.224): $(tail -1 "$here/anyangle_controlled.log")"
say "AFTER DONE"
