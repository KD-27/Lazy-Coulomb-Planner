#!/bin/bash
# Second night: driving in the generated worlds, then with moving obstacles.
here=$(cd "$(dirname "$0")" && pwd)
"$here/run_worlds.sh"
"$here/run_moving.sh"
echo "$(date '+%F %T') NIGHT2 DONE" >> "$here/run.log"
