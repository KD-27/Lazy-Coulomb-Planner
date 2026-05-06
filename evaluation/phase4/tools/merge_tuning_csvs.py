#!/usr/bin/env python3
"""
Merge the 27 per-combo CSVs from Phase 4 into one master tuning_sweep.csv,
and print a summary that supports the §4.3 selection rule.

Reads:
    ~/lcp_paper_ws/results/tuning/sweep_s{step:.2f}_r{rep:.2f}_p{pts:d}.csv

Writes:
    ~/lcp_paper_ws/results/tuning_sweep.csv

Adds three columns parsed from each filename:
    step_size, repulsion_strength, initial_path_points

Summary printed to stdout:
  1. Full ranking table of all 27 combos (success rate, mean/median planning
     time, mean clearance, etc.) sorted by mean planning time. Combos with
     <90% success are flagged [FILTERED].
  2. §4.3 winner announcement (lowest mean planning time among combos with
     >=90% success).
  3. Tie-band listing (combos within 1 stddev of the winner's per-run timing
     distribution).

Usage:
    python3 merge_tuning_csvs.py
    python3 merge_tuning_csvs.py --tuning-dir ~/lcp_paper_ws/results/tuning \\
                                 --output ~/lcp_paper_ws/results/tuning_sweep.csv
"""

import argparse
import csv
import os
import re
import statistics
import sys
from pathlib import Path

# Phase 4 §4.3 selection rule constants
SUCCESS_RATE_THRESHOLD = 0.90  # combos below this are filtered out

# Filename pattern: sweep_s0.05_r0.15_p20.csv
FILENAME_RE = re.compile(
    r'^sweep_s(?P<step>\d+\.\d+)_r(?P<rep>\d+\.\d+)_p(?P<pts>\d+)\.csv$'
)


def parse_combo_from_filename(fname: str):
    """Extract (step_size, repulsion_strength, initial_path_points) from
    a filename like 'sweep_s0.05_r0.15_p20.csv'. Returns None if no match."""
    m = FILENAME_RE.match(fname)
    if not m:
        return None
    return (float(m['step']), float(m['rep']), int(m['pts']))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning-dir',
                    default='~/lcp_paper_ws/results/tuning',
                    help='Directory containing per-combo sweep_*.csv files')
    ap.add_argument('--output',
                    default='~/lcp_paper_ws/results/tuning_sweep.csv',
                    help='Path to write merged CSV')
    args = ap.parse_args()

    tuning_dir = Path(os.path.expanduser(args.tuning_dir)).resolve()
    out_path = Path(os.path.expanduser(args.output)).resolve()

    if not tuning_dir.is_dir():
        print(f"ERROR: tuning dir not found: {tuning_dir}", file=sys.stderr)
        sys.exit(1)

    combo_csvs = sorted(tuning_dir.glob('sweep_*.csv'))
    if len(combo_csvs) != 27:
        print(f"ERROR: expected 27 sweep_*.csv files in {tuning_dir}, "
              f"found {len(combo_csvs)}", file=sys.stderr)
        sys.exit(1)

    # ------------------------------------------------------------------
    # Read each combo's CSV, append three param columns, accumulate rows
    # ------------------------------------------------------------------
    all_rows = []
    base_fieldnames = None
    combo_stats = []  # one dict per combo for the summary

    for csv_path in combo_csvs:
        combo = parse_combo_from_filename(csv_path.name)
        if combo is None:
            print(f"ERROR: cannot parse combo from filename '{csv_path.name}'",
                  file=sys.stderr)
            sys.exit(1)
        step, rep, pts = combo

        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            if base_fieldnames is None:
                base_fieldnames = reader.fieldnames
            elif reader.fieldnames != base_fieldnames:
                print(f"ERROR: column mismatch in {csv_path.name}", file=sys.stderr)
                print(f"  expected: {base_fieldnames}", file=sys.stderr)
                print(f"  got:      {reader.fieldnames}", file=sys.stderr)
                sys.exit(1)
            rows = list(reader)

        if len(rows) != 10:
            print(f"WARNING: {csv_path.name} has {len(rows)} rows, expected 10",
                  file=sys.stderr)

        # Augment each row with the three param columns
        for r in rows:
            r['step_size'] = step
            r['repulsion_strength'] = rep
            r['initial_path_points'] = pts
            all_rows.append(r)

        # Per-combo stats
        successes = [r for r in rows if r['success'] == 'True']
        success_rate = len(successes) / len(rows) if rows else 0.0
        plan_times = [float(r['planning_time_ms']) for r in successes]
        clearances = [float(r['min_clearance_m']) for r in successes]

        combo_stats.append({
            'step_size': step,
            'repulsion_strength': rep,
            'initial_path_points': pts,
            'n_total': len(rows),
            'n_success': len(successes),
            'success_rate': success_rate,
            'mean_plan_ms': statistics.mean(plan_times) if plan_times else float('nan'),
            'median_plan_ms': statistics.median(plan_times) if plan_times else float('nan'),
            'stdev_plan_ms': statistics.stdev(plan_times) if len(plan_times) >= 2 else 0.0,
            'mean_min_clearance_m': statistics.mean(clearances) if clearances else float('nan'),
            'plan_times': plan_times,  # kept for tie-band stddev computation
            'filtered': success_rate < SUCCESS_RATE_THRESHOLD,
        })

    # ------------------------------------------------------------------
    # Write merged CSV: original 21 columns + 3 param columns
    # ------------------------------------------------------------------
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_fieldnames = list(base_fieldnames) + [
        'step_size', 'repulsion_strength', 'initial_path_points'
    ]

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=out_fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)

    expected_rows = 27 * 10
    if len(all_rows) != expected_rows:
        print(f"WARNING: merged CSV has {len(all_rows)} rows, expected "
              f"{expected_rows}", file=sys.stderr)

    print(f"Wrote {len(all_rows)} rows ({len(out_fieldnames)} columns) to "
          f"{out_path}")
    print()

    # ------------------------------------------------------------------
    # Summary 1: full ranking table (sorted by mean_plan_ms, filtered marked)
    # ------------------------------------------------------------------
    print("=" * 100)
    print("Phase 4 — All 27 combos, sorted by mean planning time (ascending)")
    print("=" * 100)
    print(f"{'rank':>4}  {'step':>5} {'rep':>5} {'pts':>4}  "
          f"{'success':>9}  {'mean_ms':>8} {'med_ms':>8} {'std_ms':>7}  "
          f"{'mean_clear_cm':>14}  {'flag':<10}")
    print("-" * 100)

    sorted_combos = sorted(combo_stats, key=lambda c: c['mean_plan_ms'])
    for i, c in enumerate(sorted_combos, 1):
        flag = '[FILTERED]' if c['filtered'] else ''
        print(f"{i:>4}  "
              f"{c['step_size']:>5.2f} {c['repulsion_strength']:>5.2f} "
              f"{c['initial_path_points']:>4d}  "
              f"{c['n_success']:>3d}/{c['n_total']:<3d} "
              f"({c['success_rate']*100:>3.0f}%)  "
              f"{c['mean_plan_ms']:>8.2f} "
              f"{c['median_plan_ms']:>8.2f} "
              f"{c['stdev_plan_ms']:>7.2f}  "
              f"{c['mean_min_clearance_m']*100:>14.2f}  "
              f"{flag:<10}")

    # ------------------------------------------------------------------
    # Summary 2: §4.3 winner announcement
    # ------------------------------------------------------------------
    print()
    print("=" * 100)
    print(f"Phase 4 §4.3 selection rule")
    print(f"  Step 1: Filter combos with success rate < {SUCCESS_RATE_THRESHOLD*100:.0f}%")
    print(f"  Step 2: Among survivors, pick combo with lowest mean planning time")
    print("=" * 100)

    survivors = [c for c in sorted_combos if not c['filtered']]
    n_filtered = len(combo_stats) - len(survivors)
    print(f"Combos surviving step 1: {len(survivors)}/27 ({n_filtered} filtered)")

    if not survivors:
        print()
        print("ERROR: no combos survived the success-rate filter.")
        print("       Phase 4 cannot select a winner. Re-examine the sweep.")
        sys.exit(2)

    winner = survivors[0]  # already sorted by mean_plan_ms ascending
    print()
    print(f"§4.3 winner:")
    print(f"  step_size           = {winner['step_size']}")
    print(f"  repulsion_strength  = {winner['repulsion_strength']}")
    print(f"  initial_path_points = {winner['initial_path_points']}")
    print(f"  mean planning time  = {winner['mean_plan_ms']:.2f} ms "
          f"(stddev {winner['stdev_plan_ms']:.2f} ms over "
          f"{winner['n_success']} successful runs)")
    print(f"  success rate        = {winner['success_rate']*100:.0f}%")

    # ------------------------------------------------------------------
    # Summary 3: tie-band — combos within 1 stddev of winner's per-run dist
    # ------------------------------------------------------------------
    # Use the WINNER's per-run stddev as the tie-band width. Any combo whose
    # mean falls within winner_mean +/- winner_stddev is statistically hard
    # to distinguish from the winner.
    band_width = winner['stdev_plan_ms']
    band_low = winner['mean_plan_ms']
    band_high = winner['mean_plan_ms'] + band_width

    ties = [c for c in survivors[1:]  # skip winner itself
            if c['mean_plan_ms'] <= band_high]

    print()
    print(f"Tie band: combos with mean planning time in "
          f"[{band_low:.2f}, {band_high:.2f}] ms "
          f"(within 1 stddev of winner's per-run distribution)")

    if not ties:
        print(f"  No other combos within tie band — winner is statistically distinct.")
    else:
        print(f"  {len(ties)} other combos in the tie band (consider for override):")
        for c in ties:
            delta = c['mean_plan_ms'] - winner['mean_plan_ms']
            print(f"    step={c['step_size']:>4.2f} rep={c['repulsion_strength']:>4.2f} "
                  f"pts={c['initial_path_points']:<3d}  "
                  f"mean={c['mean_plan_ms']:>6.2f} ms  "
                  f"(+{delta:>5.2f} ms vs winner, "
                  f"success {c['success_rate']*100:.0f}%)")

    print()
    print("Next step: review the ranking, decide whether to override the §4.3 winner,")
    print("           then update config/nav2_params_lazy_coulomb.yaml with the chosen values.")


if __name__ == '__main__':
    main()
