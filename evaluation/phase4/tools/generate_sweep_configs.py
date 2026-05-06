#!/usr/bin/env python3
"""
Generate the 27 LCP tuning-sweep config YAMLs for Phase 4.

Reads the canonical nav2_params_lazy_coulomb.yaml, then writes 27 copies
to <out_dir>/, each with a different (step_size, repulsion_strength,
initial_path_points) triple patched into
planner_server.ros__parameters.GridBased.

Filename convention: sweep_s{step:.2f}_r{rep:.2f}_p{pts:d}.yaml
  e.g. sweep_s0.05_r0.15_p20.yaml

Prints the 27 absolute filenames (one per line) to stdout so the bash
driver can capture them with mapfile/readarray.

Usage:
    python3 generate_sweep_configs.py \
        --base-yaml ~/lcp_paper_ws/src/lcp_benchmark/config/nav2_params_lazy_coulomb.yaml \
        --out-dir /tmp/lcp_tuning/configs
"""

import argparse
import copy
import itertools
import os
import sys
from pathlib import Path

import yaml


# Phase 4 §4.1 grid
STEP_SIZES = [0.02, 0.05, 0.1]
REPULSION_STRENGTHS = [0.05, 0.15, 0.3]
INITIAL_PATH_POINTS = [10, 20, 40]


def patch_config(base: dict, step: float, rep: float, pts: int) -> dict:
    """Return a deep copy of base with the three LCP params patched."""
    cfg = copy.deepcopy(base)
    try:
        gb = cfg['planner_server']['ros__parameters']['GridBased']
    except KeyError as e:
        raise KeyError(
            f"Base YAML missing expected key path "
            f"planner_server.ros__parameters.GridBased: {e}"
        )
    gb['step_size'] = step
    gb['repulsion_strength'] = rep
    gb['initial_path_points'] = pts
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base-yaml', required=True,
                    help='Path to canonical nav2_params_lazy_coulomb.yaml')
    ap.add_argument('--out-dir', default='/tmp/lcp_tuning/configs',
                    help='Output directory for generated configs '
                         '(default: /tmp/lcp_tuning/configs)')
    args = ap.parse_args()

    base_path = Path(os.path.expanduser(args.base_yaml)).resolve()
    out_dir = Path(os.path.expanduser(args.out_dir)).resolve()

    if not base_path.is_file():
        print(f"ERROR: base YAML not found: {base_path}", file=sys.stderr)
        sys.exit(1)

    with open(base_path, 'r') as f:
        base_cfg = yaml.safe_load(f)

    # Sanity-check structure before generating 27 broken files
    try:
        gb = base_cfg['planner_server']['ros__parameters']['GridBased']
        for key in ('step_size', 'repulsion_strength', 'initial_path_points'):
            if key not in gb:
                print(f"ERROR: base YAML missing GridBased.{key}", file=sys.stderr)
                sys.exit(1)
    except (KeyError, TypeError) as e:
        print(f"ERROR: base YAML structure unexpected: {e}", file=sys.stderr)
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)

    written = []
    for step, rep, pts in itertools.product(
            STEP_SIZES, REPULSION_STRENGTHS, INITIAL_PATH_POINTS):
        cfg = patch_config(base_cfg, step, rep, pts)
        fname = f"sweep_s{step:.2f}_r{rep:.2f}_p{pts:d}.yaml"
        fpath = out_dir / fname
        with open(fpath, 'w') as f:
            yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
        written.append(str(fpath))

    if len(written) != 27:
        print(f"ERROR: expected 27 configs, wrote {len(written)}", file=sys.stderr)
        sys.exit(1)

    # Emit filenames to stdout for the bash driver to consume.
    # Use stderr for any human-facing summary.
    for fp in written:
        print(fp)
    print(f"# Wrote {len(written)} configs to {out_dir}", file=sys.stderr)


if __name__ == '__main__':
    main()
