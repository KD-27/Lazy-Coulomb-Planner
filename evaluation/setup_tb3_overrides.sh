#!/usr/bin/env bash
# evaluation/setup_tb3_overrides.sh
#
# Applies the TurtleBot3 source overrides required to reproduce Phase 2 maps:
#   1. LDS-01 lidar max range: 3.5 m -> 30.0 m
#   2. teleop BURGER_MAX_LIN_VEL: 0.22 m/s -> 1.0 m/s
#
# Why this script exists:
#   The edits live in vendored TB3 packages in src/, not in this repo's
#   tracked source. A fresh clone of the workspace will lack them and
#   the maps won't reproduce. Run this once after cloning + vendoring
#   the TB3 packages.
#
# Idempotent: detects if edits are already applied and exits cleanly.
#
# Usage:
#   cd ~/lcp_paper_ws
#   ./src/lazy_coulomb_planner/evaluation/setup_tb3_overrides.sh
#
# Requirements:
#   - WORKSPACE env var (defaults to ~/lcp_paper_ws if unset)
#   - turtlebot3_simulations and turtlebot3 packages already in src/

set -euo pipefail

# ---- config ----------------------------------------------------------------

WORKSPACE="${WORKSPACE:-$HOME/lcp_paper_ws}"
LIDAR_SDF="$WORKSPACE/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
TELEOP_PY="$WORKSPACE/src/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py"

# ---- helpers ---------------------------------------------------------------

info() { printf '\033[0;34m[info]\033[0m %s\n' "$*"; }
ok()   { printf '\033[0;32m[ ok ]\033[0m %s\n' "$*"; }
warn() { printf '\033[0;33m[warn]\033[0m %s\n' "$*"; }
err()  { printf '\033[0;31m[err ]\033[0m %s\n' "$*" >&2; }

# ---- preflight -------------------------------------------------------------

info "Workspace: $WORKSPACE"

if [[ ! -f "$LIDAR_SDF" ]]; then
    err "Lidar SDF not found: $LIDAR_SDF"
    err "Make sure turtlebot3_simulations is checked out in src/"
    exit 1
fi

if [[ ! -f "$TELEOP_PY" ]]; then
    err "Teleop file not found: $TELEOP_PY"
    err "Make sure turtlebot3 (teleop) is checked out in src/"
    exit 1
fi

# ---- edit 1: lidar range ---------------------------------------------------

info "Checking lidar range in model.sdf ..."

if grep -q '<max>30.0</max>' "$LIDAR_SDF"; then
    ok "Lidar range already set to 30.0 m (skipping)"
elif grep -q '<max>3.5</max>' "$LIDAR_SDF"; then
    info "Editing lidar range 3.5 -> 30.0 m"
    sed -i 's|<max>3.5</max>|<max>30.0</max>|' "$LIDAR_SDF"
    if grep -q '<max>30.0</max>' "$LIDAR_SDF"; then
        ok "Lidar range edit applied"
    else
        err "Lidar edit did not apply as expected"
        exit 1
    fi
else
    warn "Lidar SDF has neither '<max>3.5</max>' nor '<max>30.0</max>'."
    warn "Manual inspection required: $LIDAR_SDF"
    exit 1
fi

# ---- edit 2: teleop max linear velocity -----------------------------------

info "Checking teleop max linear velocity ..."

if grep -q '^BURGER_MAX_LIN_VEL = 1\.0' "$TELEOP_PY"; then
    ok "Teleop velocity already set to 1.0 m/s (skipping)"
elif grep -q '^BURGER_MAX_LIN_VEL = 0\.22' "$TELEOP_PY"; then
    info "Editing BURGER_MAX_LIN_VEL 0.22 -> 1.0"
    sed -i 's|BURGER_MAX_LIN_VEL = 0\.22|BURGER_MAX_LIN_VEL = 1.0|' "$TELEOP_PY"
    if grep -q '^BURGER_MAX_LIN_VEL = 1\.0' "$TELEOP_PY"; then
        ok "Teleop velocity edit applied"
    else
        err "Teleop edit did not apply as expected"
        exit 1
    fi
else
    warn "Teleop file has neither 'BURGER_MAX_LIN_VEL = 0.22' nor '= 1.0'."
    warn "Manual inspection required: $TELEOP_PY"
    exit 1
fi

# ---- rebuild affected packages --------------------------------------------
# --symlink-install does not reliably propagate manual edits to non-Python
# files (model.sdf in particular). Force a clean rebuild of both packages.

info "Force-rebuilding turtlebot3_gazebo and turtlebot3_teleop ..."

cd "$WORKSPACE"
rm -rf build/turtlebot3_gazebo install/turtlebot3_gazebo
rm -rf build/turtlebot3_teleop install/turtlebot3_teleop

if ! colcon build --packages-select turtlebot3_gazebo turtlebot3_teleop --symlink-install; then
    err "colcon build failed"
    exit 1
fi

ok "Build complete"

# ---- verify install/ has the edits ----------------------------------------

info "Verifying edits propagated to install/ ..."

INSTALL_SDF="$WORKSPACE/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
if [[ -f "$INSTALL_SDF" ]] && grep -q '<max>30.0</max>' "$INSTALL_SDF"; then
    ok "install/ lidar SDF has 30.0 m range"
else
    err "install/ lidar SDF missing or wrong: $INSTALL_SDF"
    exit 1
fi

# Teleop install path varies; the egg-link points to src for --symlink-install.
EGG_LINK="$WORKSPACE/install/turtlebot3_teleop/lib/python3.10/site-packages/turtlebot3-teleop.egg-link"
if [[ -f "$EGG_LINK" ]]; then
    SRC_PATH="$(cat "$EGG_LINK" | head -1)"
    if grep -q 'BURGER_MAX_LIN_VEL = 1\.0' "$SRC_PATH/turtlebot3_teleop/script/teleop_keyboard.py"; then
        ok "Teleop source (via egg-link) has 1.0 m/s"
    else
        err "Teleop source missing edit at: $SRC_PATH"
        exit 1
    fi
else
    warn "Egg-link not found, checking direct install path ..."
    INSTALL_TELEOP="$WORKSPACE/install/turtlebot3_teleop/lib/python3.10/site-packages/turtlebot3_teleop/script/teleop_keyboard.py"
    if [[ -f "$INSTALL_TELEOP" ]] && grep -q 'BURGER_MAX_LIN_VEL = 1\.0' "$INSTALL_TELEOP"; then
        ok "install/ teleop has 1.0 m/s"
    else
        err "Could not verify teleop install"
        exit 1
    fi
fi

# ---- done -----------------------------------------------------------------

ok "All TB3 overrides applied and verified"
info "Remember to 'source $WORKSPACE/install/setup.bash' in fresh shells"
