#!/usr/bin/env bash
#
# LCP Phase 4 tuning sweep driver.
#
# Workflow:
#   1. (one-time) Generate 10 (start, goal) seeds on event_venue.
#   2. (one-time) Generate the 27 tuning config YAMLs via generate_sweep_configs.py.
#   3. (preflight) Run 2 extreme combos x 2 seeds = 4 runs. Halt if anything looks wrong.
#   4. (sweep)    For each of 27 configs: bring up Nav2 with that config, run harness
#                 with 10 seeds, write to per-combo CSV.
#   5. (after)    File 3 (merge_tuning_csvs.py) combines per-combo CSVs into one.
#
# Behavior: FAIL-FAST. First non-zero exit from any subcommand halts the script.
#
# Usage:
#   run_tuning_sweep.sh --preflight-only   # just runs the 4 preflight runs and exits
#   run_tuning_sweep.sh                    # runs preflight, then if it passes, the 270
#
# Output layout:
#   ~/lcp_paper_ws/results/tuning/
#       preflight_event_venue.csv          (4 rows after preflight)
#       sweep_s0.02_r0.05_p10.csv          (10 rows per combo)
#       ... x 27
#       seeds_event_venue_tuning.yaml      (10 seeds, generated once)
#
# -----------------------------------------------------------------------------
# Phase 4 known issues / watch-items (logged during preflight, May 2026):
#
#   * min_clearance_m observed ~0.7-1.3 cm on event_venue successful paths.
#     LCP's planning succeeds but paths near-clip obstacles. Not blocking
#     for Phase 4 selection (criteria use success/time only). Address in
#     paper's Discussion section.
#
#   * cumulative_curvature = 0.0 on event_venue (confirmed metrics.py bug;
#     Phase 3 §3.9 watch-item #1 escalated). Does not block Phase 4 since
#     selection criteria don't use curvature. MUST FIX before Phase 6
#     statistical analysis.
#
# -----------------------------------------------------------------------------
# Process-management notes (lessons from preflight runs):
#   * `ros2 launch` spawns a process group. SIGINT to the launch parent does
#     NOT cascade to children. Must use setsid + `kill -- -PGID` to signal
#     the entire group. Same lesson as Manriix CRSF teleop teardown.
#   * Stale lifecycle_manager / planner_server / map_server processes from
#     prior runs will silently shadow this sweep's bringup, producing
#     wrong-config results with no error. Strict precondition check at start.

set -euo pipefail

# -----------------------------------------------------------------------------
# Hardcoded paths (Phase 4 specific tool, deployed env)
# -----------------------------------------------------------------------------
WS_ROOT="${HOME}/lcp_paper_ws"
HARNESS_PKG_CONFIG="${WS_ROOT}/src/lcp_benchmark/config"
MAPS_DIR="${WS_ROOT}/maps"
TOOLS_DIR="${WS_ROOT}/evaluation/phase4/tools"

BASE_LCP_YAML="${HARNESS_PKG_CONFIG}/nav2_params_lazy_coulomb.yaml"
EVENT_VENUE_MAP="${MAPS_DIR}/event_venue.yaml"

CONFIGS_DIR="/tmp/lcp_tuning/configs"
RESULTS_DIR="${WS_ROOT}/results/tuning"
SEEDS_FILE="${RESULTS_DIR}/seeds_event_venue_tuning.yaml"
PREFLIGHT_CSV="${RESULTS_DIR}/preflight_event_venue.csv"

NUM_SEEDS=10
SEED_BASE=0           # so seed indices are 0..9 (matches existing convention in pose_sampler)
BRINGUP_WAIT_S=8      # matches smoke_test.launch.py
TEARDOWN_WAIT_S=2     # gap between combos so processes/ports clear cleanly
PLAN_TIMEOUT_S=30.0
TEARDOWN_TIMEOUT_S=10 # how long to wait for SIGINT to gracefully kill the process group

# Preflight extremes (the two corners of the parameter cube)
PREFLIGHT_CONFIGS=(
    "${CONFIGS_DIR}/sweep_s0.02_r0.05_p10.yaml"
    "${CONFIGS_DIR}/sweep_s0.10_r0.30_p40.yaml"
)
PREFLIGHT_SEEDS=2

mkdir -p /tmp/lcp_tuning  # for bringup logs

# -----------------------------------------------------------------------------
# Arg parsing
# -----------------------------------------------------------------------------
PREFLIGHT_ONLY=0
if [ "${1:-}" = "--preflight-only" ]; then
    PREFLIGHT_ONLY=1
elif [ -n "${1:-}" ]; then
    echo "ERROR: unknown argument '$1'" >&2
    echo "Usage: $0 [--preflight-only]" >&2
    exit 2
fi

# -----------------------------------------------------------------------------
# Sanity checks
# -----------------------------------------------------------------------------
echo "=== Phase 4 sanity checks ==="
for path in "${BASE_LCP_YAML}" "${EVENT_VENUE_MAP}" "${TOOLS_DIR}/generate_sweep_configs.py"; do
    if [ ! -f "${path}" ]; then
        echo "ERROR: required file missing: ${path}" >&2
        exit 1
    fi
done

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not on PATH. Source your workspace first." >&2
    exit 1
fi

# Strict precondition: no stale Nav2 processes allowed.
echo "=== Checking for stale Nav2 processes ==="
STALE=$(pgrep -af "nav2_lifecycle_manager|nav2_planner|nav2_map_server" || true)
if [ -n "${STALE}" ]; then
    echo "ERROR: stale Nav2 processes detected. Clean them up before re-running:" >&2
    echo "${STALE}" >&2
    echo "" >&2
    echo "  Suggested cleanup:" >&2
    echo "    pkill -9 -f 'lifecycle_manager|planner_server|map_server'" >&2
    echo "    sleep 1" >&2
    echo "    ps -ef | grep -E 'ros2|nav2' | grep -v grep" >&2
    exit 1
fi
echo "  -> no stale processes ✓"

mkdir -p "${RESULTS_DIR}"

# -----------------------------------------------------------------------------
# 1. Generate the 27 sweep configs (idempotent)
# -----------------------------------------------------------------------------
echo "=== Generating 27 sweep configs ==="
python3 "${TOOLS_DIR}/generate_sweep_configs.py" \
    --base-yaml "${BASE_LCP_YAML}" \
    --out-dir "${CONFIGS_DIR}" >/dev/null

ACTUAL_CONFIGS=$(ls "${CONFIGS_DIR}"/sweep_*.yaml | wc -l)
if [ "${ACTUAL_CONFIGS}" -ne 27 ]; then
    echo "ERROR: expected 27 configs in ${CONFIGS_DIR}, found ${ACTUAL_CONFIGS}" >&2
    exit 1
fi
echo "  -> 27 configs in ${CONFIGS_DIR}"

# -----------------------------------------------------------------------------
# 2. Generate seeds file once (10 pairs on event_venue), shared by all combos
# -----------------------------------------------------------------------------
if [ ! -f "${SEEDS_FILE}" ]; then
    echo "=== Generating ${NUM_SEEDS} seeds on event_venue ==="
    ros2 run lcp_benchmark generate_seeds \
        "${EVENT_VENUE_MAP}" "${SEEDS_FILE}" "${NUM_SEEDS}" "${SEED_BASE}"
else
    echo "=== Reusing existing seeds file: ${SEEDS_FILE} ==="
fi

SEED_COUNT=$(python3 -c "
import yaml, sys
with open('${SEEDS_FILE}') as f:
    d = yaml.safe_load(f)
print(len(d.get('pairs', [])))
")
if [ "${SEED_COUNT}" -ne "${NUM_SEEDS}" ]; then
    echo "ERROR: seeds file has ${SEED_COUNT} pairs, expected ${NUM_SEEDS}" >&2
    exit 1
fi
echo "  -> seeds file has ${NUM_SEEDS} pairs"

# -----------------------------------------------------------------------------
# Helper: bring down a process group cleanly using SIGINT, then SIGKILL fallback.
# Argument: the process group leader's PID (== PGID when launched with setsid).
# -----------------------------------------------------------------------------
teardown_process_group() {
    local pgid="$1"

    if ! kill -0 -- "-${pgid}" 2>/dev/null; then
        return 0
    fi

    kill -INT -- "-${pgid}" 2>/dev/null || true

    local waited=0
    while [ "${waited}" -lt "${TEARDOWN_TIMEOUT_S}" ]; do
        if ! kill -0 -- "-${pgid}" 2>/dev/null; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done

    echo "  WARN: process group ${pgid} did not exit on SIGINT after ${TEARDOWN_TIMEOUT_S}s, sending SIGKILL" >&2
    kill -9 -- "-${pgid}" 2>/dev/null || true
    sleep 1
    return 0
}

# -----------------------------------------------------------------------------
# Helper: run harness once with a given (config, seeds_file, output_csv).
#
# Args:
#   $1 = config YAML path
#   $2 = seeds YAML path
#   $3 = output CSV path
#   $4 = combo label (used for log filename)
#   $5 = "wipe" or "append" — whether to delete an existing CSV before running
# -----------------------------------------------------------------------------
run_one_combo() {
    local config_yaml="$1"
    local seeds_path="$2"
    local output_csv="$3"
    local combo_label="$4"
    local csv_mode="$5"

    if [ "${csv_mode}" = "wipe" ]; then
        rm -f "${output_csv}"
    elif [ "${csv_mode}" != "append" ]; then
        echo "ERROR: run_one_combo csv_mode must be 'wipe' or 'append', got '${csv_mode}'" >&2
        return 2
    fi

    echo "  [${combo_label}] bringing up Nav2 stack..."
    # setsid puts ros2 launch in its own process group so we can signal the
    # whole group (parent + children) atomically during teardown.
    setsid ros2 launch lcp_benchmark lcp_planner_only.launch.py \
        params_file:="${config_yaml}" \
        map:="${EVENT_VENUE_MAP}" \
        > "/tmp/lcp_tuning/bringup_${combo_label}.log" 2>&1 &
    local bringup_pid=$!
    local bringup_pgid="${bringup_pid}"  # setsid sets PGID == PID of session leader

    # Trap ensures we tear down the process group on Ctrl-C / errors.
    trap 'echo "  [${combo_label}] interrupted, tearing down..."; teardown_process_group '"${bringup_pgid}"'; trap - EXIT INT TERM; exit 130' EXIT INT TERM

    sleep "${BRINGUP_WAIT_S}"

    if ! kill -0 "${bringup_pid}" 2>/dev/null; then
        echo "ERROR: [${combo_label}] bringup died during startup. See /tmp/lcp_tuning/bringup_${combo_label}.log" >&2
        trap - EXIT INT TERM
        return 1
    fi

    echo "  [${combo_label}] running harness..."
    set +e
    ros2 run lcp_benchmark harness \
        --ros-args \
        -p "output_csv:=${output_csv}" \
        -p "planner_name:=LazyCoulomb" \
        -p "map_name:=event_venue" \
        -p "map_yaml:=${EVENT_VENUE_MAP}" \
        -p "seeds_file:=${seeds_path}" \
        -p "plan_timeout_s:=${PLAN_TIMEOUT_S}"
    local harness_rc=$?
    set -e

    echo "  [${combo_label}] tearing down Nav2 stack..."
    teardown_process_group "${bringup_pgid}"
    trap - EXIT INT TERM

    sleep "${TEARDOWN_WAIT_S}"

    if [ "${harness_rc}" -ne 0 ]; then
        echo "ERROR: [${combo_label}] harness exited non-zero (${harness_rc})" >&2
        return "${harness_rc}"
    fi

    if [ ! -f "${output_csv}" ]; then
        echo "ERROR: [${combo_label}] no CSV produced at ${output_csv}" >&2
        return 1
    fi

    return 0
}

# -----------------------------------------------------------------------------
# 3. Pre-flight: 2 extreme combos x 2 seeds, accumulated into ONE CSV
# -----------------------------------------------------------------------------
echo "=== Pre-flight check (2 combos x 2 seeds on event_venue) ==="

PREFLIGHT_SEEDS_FILE="${RESULTS_DIR}/seeds_preflight.yaml"
python3 -c "
import yaml
with open('${SEEDS_FILE}') as f:
    d = yaml.safe_load(f)
d['pairs'] = d['pairs'][:${PREFLIGHT_SEEDS}]
with open('${PREFLIGHT_SEEDS_FILE}', 'w') as f:
    yaml.safe_dump(d, f, default_flow_style=False)
"

# Wipe once before the loop, then both combos APPEND to the same file
rm -f "${PREFLIGHT_CSV}"

FIRST=1
for cfg in "${PREFLIGHT_CONFIGS[@]}"; do
    label=$(basename "${cfg}" .yaml)
    if [ "${FIRST}" -eq 1 ]; then
        run_one_combo "${cfg}" "${PREFLIGHT_SEEDS_FILE}" "${PREFLIGHT_CSV}" "preflight-${label}" "wipe"
        FIRST=0
    else
        run_one_combo "${cfg}" "${PREFLIGHT_SEEDS_FILE}" "${PREFLIGHT_CSV}" "preflight-${label}" "append"
    fi
done

PREFLIGHT_ROWS=$(($(wc -l < "${PREFLIGHT_CSV}") - 1))
EXPECTED_PREFLIGHT_ROWS=$(( ${#PREFLIGHT_CONFIGS[@]} * PREFLIGHT_SEEDS ))
echo "=== Pre-flight produced ${PREFLIGHT_ROWS} rows (expected ${EXPECTED_PREFLIGHT_ROWS}) ==="

if [ "${PREFLIGHT_ROWS}" -ne "${EXPECTED_PREFLIGHT_ROWS}" ]; then
    echo "ERROR: preflight row count mismatch. Inspect ${PREFLIGHT_CSV}" >&2
    exit 1
fi

PREFLIGHT_SUMMARY=$(python3 -c "
import csv
from collections import defaultdict
with open('${PREFLIGHT_CSV}') as f:
    rows = list(csv.DictReader(f))
total_success = sum(1 for r in rows if r['success'] == 'True')
reasons = defaultdict(int)
for r in rows:
    if r['success'] != 'True':
        reasons[r['failure_reason']] += 1
print(f'total_success={total_success}/{len(rows)}')
for reason, count in reasons.items():
    print(f'  failure_reason \"{reason}\": {count}')
")
echo "${PREFLIGHT_SUMMARY}"

NONZERO_CURV=$(python3 -c "
import csv
with open('${PREFLIGHT_CSV}') as f:
    rows = list(csv.DictReader(f))
nz = sum(1 for r in rows
         if r['success'] == 'True'
         and r.get('cumulative_curvature', '0') not in ('', '0', '0.0', '0.0000'))
print(nz)
")
if [ "${NONZERO_CURV}" -eq 0 ]; then
    echo "WARNING: cumulative_curvature is 0 on all successful preflight runs." >&2
    echo "         Confirmed metrics.py bug (Phase 3 §3.9 watch-item #1 escalated)." >&2
    echo "         Phase 4 selection criteria don't use curvature, so sweep can proceed." >&2
    echo "         MUST FIX before Phase 6 statistical analysis." >&2
else
    echo "  -> nonzero curvature observed on ${NONZERO_CURV} successful run(s) ✓"
fi

if [ "${PREFLIGHT_ONLY}" -eq 1 ]; then
    echo ""
    echo "=== --preflight-only specified, exiting before full sweep ==="
    echo "Inspect: ${PREFLIGHT_CSV}"
    exit 0
fi

# -----------------------------------------------------------------------------
# 4. Full tuning sweep: 27 combos x 10 seeds = 270 runs
# -----------------------------------------------------------------------------
echo ""
echo "=== Full tuning sweep: 27 combos x ${NUM_SEEDS} seeds = $((27 * NUM_SEEDS)) runs ==="

START_TS=$(date +%s)
COMBO_INDEX=0

for cfg in "${CONFIGS_DIR}"/sweep_*.yaml; do
    COMBO_INDEX=$((COMBO_INDEX + 1))
    label=$(basename "${cfg}" .yaml)
    output_csv="${RESULTS_DIR}/${label}.csv"
    echo ""
    echo "--- [${COMBO_INDEX}/27] ${label} ---"
    run_one_combo "${cfg}" "${SEEDS_FILE}" "${output_csv}" "${label}" "wipe"

    rows=$(($(wc -l < "${output_csv}") - 1))
    if [ "${rows}" -ne "${NUM_SEEDS}" ]; then
        echo "ERROR: [${label}] produced ${rows} rows, expected ${NUM_SEEDS}" >&2
        exit 1
    fi
    echo "  -> ${rows} rows written to ${output_csv}"
done

END_TS=$(date +%s)
ELAPSED=$((END_TS - START_TS))

echo ""
echo "=== Sweep complete: 27 combos in $((ELAPSED / 60))m $((ELAPSED % 60))s ==="
echo "Results: ${RESULTS_DIR}/sweep_*.csv"
echo "Next step: run merge_tuning_csvs.py to combine into tuning_sweep.csv"
