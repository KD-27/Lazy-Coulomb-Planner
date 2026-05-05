#!/usr/bin/env python3
"""
Phase 3 smoke test sanity check.
Plots the 3 smoke (start, goal) pairs over the empty.pgm map.
"""
import csv
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
import yaml

CSV_PATH = Path.home() / "lcp_paper_ws/results/smoke_test.csv"
MAP_PGM  = Path.home() / "lcp_paper_ws/maps/empty.pgm"
MAP_YAML = Path.home() / "lcp_paper_ws/maps/empty.yaml"
OUT_PNG  = Path.home() / "lcp_paper_ws/src/lazy_coulomb_planner/evaluation/phase3/smoke_path_check.png"

# 1. Load map metadata
with open(MAP_YAML) as f:
    meta = yaml.safe_load(f)
res = meta["resolution"]
origin = meta["origin"]   # [origin_x, origin_y, theta]
print(f"Map resolution: {res} m/cell")
print(f"Map origin: ({origin[0]}, {origin[1]}) m")

# 2. Load PGM image
img = np.array(Image.open(MAP_PGM))
h, w = img.shape
print(f"Map image: {w} x {h} px = {w*res:.1f} m x {h*res:.1f} m")

# 3. World extent in metres (for imshow extent param)
extent = [
    origin[0],
    origin[0] + w * res,
    origin[1],
    origin[1] + h * res,
]

# 4. Load CSV rows
rows = []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)
print(f"Loaded {len(rows)} smoke runs")

# 5. Plot
fig, ax = plt.subplots(figsize=(8, 8))
# PGM convention: 0=black=obstacle, 254=light_grey=free, ~205=grey=unknown.
# Display as greyscale, origin="lower" because PGM y-axis is flipped vs world.
ax.imshow(img, cmap="gray", origin="lower", extent=extent, vmin=0, vmax=255)

colors = ["tab:blue", "tab:orange", "tab:green"]
for i, r in enumerate(rows):
    sx, sy = float(r["start_x"]), float(r["start_y"])
    gx, gy = float(r["goal_x"]), float(r["goal_y"])
    seed = int(r["seed"])
    pl = float(r["path_length_m"])
    sld = float(r["straight_line_dist"])
    c = colors[i % len(colors)]
    ax.plot([sx, gx], [sy, gy], "--", color=c, alpha=0.6, linewidth=1.5)
    ax.plot(sx, sy, "o", color=c, markersize=10, markeredgecolor="black",
            label=f"seed={seed}: start ({sx:.1f},{sy:.1f})")
    ax.plot(gx, gy, "s", color=c, markersize=10, markeredgecolor="black")
    midx, midy = (sx+gx)/2, (sy+gy)/2
    ax.annotate(f"  s{seed}\n  L={pl:.2f}m\n  SL={sld:.2f}m",
                (midx, midy), fontsize=8, color=c)

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Smoke test sanity check: NavFn on empty map\n"
             "circle=start  square=goal  dashed=straight-line baseline")
ax.legend(loc="upper left", fontsize=8)
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)

OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT_PNG, dpi=120, bbox_inches="tight")
print(f"Saved: {OUT_PNG}")
