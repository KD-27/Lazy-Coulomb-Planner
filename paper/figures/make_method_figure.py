#!/usr/bin/env python3
"""
Method figure (Fig. 1): the lazy correction on a small costmap.

Usage (from the repository root): python3 paper/figures/make_method_figure.py
Writes paper/figures/method_idea.pdf (and a .png preview).

The geometry is computed, not drawn by hand: a toy costmap (obstacle cells plus an
inscribed band, both blocked for the planner), an exact grid traversal for the
segment checks, the Coulomb-like repulsion of Eq. (1) for the side choice, and the
push along the segment normal to free space plus a margin. The post-processing in
panel (c) is simplified here (farthest-visible shortcut, then Chaikin smoothing with
the clear-chord check) and is meant to show the idea, not to reproduce the plugin.
"""
import math
import os

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch  # noqa: E402

here = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({
    'font.size': 8,
    'font.family': 'serif',
    'font.serif': ['STIXGeneral', 'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'axes.linewidth': 0.5,
})

# ----------------------------------------------------------------- toy costmap
W, H = 56, 28  # cells
obst = np.zeros((H, W), bool)
yy, xx = np.mgrid[0:H, 0:W]
# Obstacle A: a wall that hangs down from the line, so most of its mass lies
# below the straight line and the repulsion at the entry point pushes upwards.
obst |= (xx >= 15) & (xx <= 19) & (yy >= 6) & (yy <= 14)
obst |= ((xx - 17) ** 2 + (yy - 6) ** 2) <= 9
# Obstacle B: a round obstacle that the segment from the first waypoint to the
# goal clips low down, so its repulsion pushes the second waypoint downwards.
obst |= ((xx - 36) ** 2 + (yy - 18) ** 2) <= 16
# Obstacle C: clutter that is not on the line (no work is done there).
obst |= ((xx - 45) ** 2 + (yy - 6) ** 2) <= 6
obst |= (xx >= 6) & (xx <= 9) & (yy >= 21) & (yy <= 23)
INFL = 1.6  # inscribed band, cells
dist = np.full((H, W), 1e9)
oy, ox = np.nonzero(obst)
for y, x in zip(oy, ox):
    dist = np.minimum(dist, np.hypot(xx - x, yy - y))
blocked = dist <= INFL  # what the planner treats as an obstacle (cost >= 253)


def free(p):
    x, y = int(math.floor(p[0])), int(math.floor(p[1]))
    return 0 <= x < W and 0 <= y < H and not blocked[y, x]


def traverse(a, b):
    """Cells touched by segment a-b (cell (i, j) covers [i, i+1) x [j, j+1))."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 8) + 2
    out = []
    for t in np.linspace(0.0, 1.0, n):
        c = (int(math.floor(a[0] + t * (b[0] - a[0]))), int(math.floor(a[1] + t * (b[1] - a[1]))))
        if not out or out[-1] != c:
            out.append(c)
    return out


def first_blocked(a, b):
    for i, j in traverse(a, b):
        if not (0 <= i < W and 0 <= j < H) or blocked[j, i]:
            return np.array([i + 0.5, j + 0.5])
    return None


def clear(a, b):
    return first_blocked(a, b) is None


R = 6.0  # charge radius, cells


def repulsion(w):
    by, bx = np.nonzero(blocked)
    c = np.stack([bx + 0.5, by + 0.5], 1)
    d = w - c
    r = np.hypot(d[:, 0], d[:, 1])
    keep = r <= R
    d, r = d[keep], np.maximum(r[keep], 0.5)
    return (d / r[:, None] ** 3).sum(0), c[keep], d / r[:, None] ** 3


def push(a, b, w, margin=2.5):
    """Push w along the normal of a-b, side from the repulsion, then a margin."""
    t = (b - a) / np.linalg.norm(b - a)
    n = np.array([-t[1], t[0]])  # left normal
    F, _, _ = repulsion(w)
    side = 1.0 if F @ n >= 0 else -1.0
    p = w.copy()
    trail = [p.copy()]
    while not free(p):
        p = p + side * 0.5 * n
        trail.append(p.copy())
    for _ in range(int(margin / 0.5)):
        q = p + side * 0.5 * n
        if not free(q):
            break
        p = q
        trail.append(p.copy())
    return p, n, side, np.array(trail)


s = np.array([3.0, 14.0])
g = np.array([53.0, 14.0])

# Step 1: s-g is blocked where it enters the wall; the repulsion of the blocked
# cells within R of that point picks the side.
w1 = first_blocked(s, g)
F1, cells1, f1 = repulsion(w1)
_, n1, side1, _ = push(s, g, w1)

# Then the general loop: scan the segments in order, correct the first blocked
# one, lock the waypoint and resume with the segment that ends at it.
lazy = [s, g]
pushes = []  # (segment start, segment end, entry cell, pushed waypoint, trail, side)
locked = 0
while len(pushes) < 6:
    for k in range(locked, len(lazy) - 1):
        hit = first_blocked(lazy[k], lazy[k + 1])
        if hit is not None:
            break
    else:
        break
    wp, nk, sk, tr = push(lazy[k], lazy[k + 1], hit)
    pushes.append((lazy[k], lazy[k + 1], hit, wp, tr, sk))
    lazy.insert(k + 1, wp)
    locked = k
assert all(clear(a, b) for a, b in zip(lazy[:-1], lazy[1:])), 'lazy path not clear'
w1p, trail1 = pushes[0][3], pushes[0][4]

# Panel (c): simplification (farthest visible), then Chaikin with clear chords.
def simplify(path):
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def densify(path, step=1.0):
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        for t in np.linspace(0, 1, k + 1)[1:]:
            out.append(a + t * (b - a))
    return out


def chaikin(path, passes=5):
    p = [np.asarray(q, float) for q in path]
    for _ in range(passes):
        new = [p[0]]
        for i in range(1, len(p) - 1):
            a, b, c = p[i - 1], p[i], p[i + 1]
            q, r = 0.75 * b + 0.25 * a, 0.75 * b + 0.25 * c
            if clear(q, r) and clear(new[-1], q):
                new += [q, r]
            else:
                new.append(b)
        new.append(p[-1])
        p = new
    return np.array(p)


simple = simplify(lazy)
final = chaikin(simple)

# ------------------------------------------------------------------- drawing
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

C_OBST = '#4d4d4d'
C_INFL = '#c4c4c4'
C_GRID = '#e8e8e8'
C_LINE = '#8a8a8a'
C_PATH = '#1f5fa8'
C_PUSH = '#c0392b'
C_FORCE = '#e08a1e'

img = np.zeros((H, W))
img[blocked] = 1
img[obst] = 2
cmap = ListedColormap(['white', C_INFL, C_OBST])

fig, axes = plt.subplots(1, 3, figsize=(7.16, 1.42), gridspec_kw={'wspace': 0.035})


def base(ax, title, xlim=(0, W), ylim=(0, H)):
    ax.imshow(img, cmap=cmap, origin='lower', extent=(0, W, 0, H), interpolation='nearest',
              vmin=0, vmax=2, zorder=0)
    for x in range(W + 1):
        ax.plot([x, x], [0, H], color=C_GRID, lw=0.25, zorder=0.5)
    for y in range(H + 1):
        ax.plot([0, W], [y, y], color=C_GRID, lw=0.25, zorder=0.5)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color('#9a9a9a')
        sp.set_linewidth(0.5)
    ax.text(0.015, 0.965, title, transform=ax.transAxes, ha='left', va='top', fontsize=8,
            bbox=dict(fc='white', ec='none', pad=0.8, alpha=0.92), zorder=20)


def arrow(ax, a, b, color, lw=1.0, ms=6, z=7):
    ax.add_patch(FancyArrowPatch(tuple(a), tuple(b), arrowstyle='-|>', mutation_scale=ms, color=color,
                                 lw=lw, shrinkA=0, shrinkB=0, zorder=z))


def label(ax, p, text, color, ha='center', va='center', size=8.5):
    ax.text(p[0], p[1], text, color=color, ha=ha, va=va, fontsize=size, zorder=15,
            bbox=dict(fc='white', ec='none', pad=0.3, alpha=0.85))


def endpoints(ax):
    for p, t, ha, dx in ((s, '$s$', 'right', -0.9), (g, '$g$', 'left', 0.9)):
        ax.plot(*p, 'o', color='black', ms=3.4, zorder=12)
        ax.text(p[0] + dx, p[1], t, ha=ha, va='center', fontsize=9.5, zorder=12)


# (a) zoom on the first blocked cell: repulsion of the blocked cells within R
ax = axes[0]
zx, zy = (w1[0] - 13.0, w1[0] + 15.0), (w1[1] - 7.5, w1[1] + 6.5)
base(ax, '(a) first blocked cell', zx, zy)
ax.plot([s[0], g[0]], [s[1], g[1]], '--', color=C_LINE, lw=1.0, zorder=3)
ax.add_patch(Circle(tuple(w1), R, fill=False, ec=C_FORCE, lw=0.7, ls=(0, (3, 2)), zorder=4))
mag = np.hypot(f1[:, 0], f1[:, 1])
for c, f in zip(cells1, f1):  # one arrow per blocked cell within R, width ~ |f|
    if np.hypot(*f) == 0:  # the cell w itself is in
        continue
    rel = np.hypot(*f) / mag.max()
    d = f / np.hypot(*f)
    arrow(ax, c - 0.35 * d, c + 0.45 * d, C_FORCE, lw=0.3 + 1.2 * rel ** 0.5, ms=2.5 + 2.5 * rel ** 0.5, z=5)
Fu = F1 / np.linalg.norm(F1)
arrow(ax, w1, w1 + 3.8 * Fu, C_FORCE, lw=1.6, ms=8, z=9)
label(ax, w1 + 3.8 * Fu + np.array([-0.9, 0.0]), r'$\mathbf{F}$', C_FORCE, size=9.5)
nn = n1 * side1
arrow(ax, w1, w1 + 3.0 * nn, C_PUSH, lw=1.0, ms=6, z=9)
label(ax, w1 + 3.0 * nn + np.array([-1.0, 0.2]), r'$\mathbf{n}$', C_PUSH, size=9.5)
ax.plot(*w1, 'o', color=C_PUSH, ms=4.2, zorder=10)
label(ax, w1 + np.array([-1.3, -1.2]), '$w$', C_PUSH, size=9.5)
rp = w1 + R * np.array([math.cos(-0.6), math.sin(-0.6)])
label(ax, rp + np.array([0.9, -0.3]), '$R$', C_FORCE, size=9.5)
# the side is the sign of F.n: project F onto n
Fn = (F1 @ nn) / np.linalg.norm(F1) * 3.8
tipF = w1 + 3.8 * Fu
ax.plot([tipF[0], w1[0]], [tipF[1], w1[1] + Fn], ':', color=C_FORCE, lw=0.8, zorder=8)
ax.plot([w1[0] - 0.35, w1[0] + 0.35], [w1[1] + Fn] * 2, color=C_FORCE, lw=1.2, zorder=9)
label(ax, (w1[0] + 1.2, w1[1] + Fn + 0.55), r'$\mathbf{F}\cdot\mathbf{n}>0$', C_FORCE, ha='left', size=7.5)

# (b) push along n to free space plus a margin, lock, continue with the next segment
ax = axes[1]
base(ax, '(b) push, lock, continue')
ax.plot([s[0], g[0]], [s[1], g[1]], '--', color=C_LINE, lw=0.8, zorder=3)
for a, b, *_ in pushes[1:]:
    ax.plot([a[0], b[0]], [a[1], b[1]], '--', color=C_PATH, lw=0.8, alpha=0.75, zorder=4)
ax.plot([p[0] for p in lazy], [p[1] for p in lazy], '-', color=C_PATH, lw=1.4, zorder=6)
for idx, (_, _, entry, wp, tr, sk) in enumerate(pushes):
    ax.plot(tr[:, 0], tr[:, 1], color=C_PUSH, lw=0.9, zorder=7)
    arrow(ax, tr[-2], tr[-1], C_PUSH, lw=0.9, ms=5, z=8)
    ax.plot(*entry, 'o', mfc='white', mec=C_PUSH, mew=0.8, ms=3.0, zorder=9)
    ax.plot(*wp, 's', color=C_PUSH, ms=3.8, zorder=10)
    up = wp[1] > entry[1]
    label(ax, wp + np.array([0.0, 1.6 if up else -1.6]), '$w_%d$' % (idx + 1), C_PUSH,
          va='bottom' if up else 'top', size=9)
    # the other side, kept for backtracking
    arrow(ax, entry, entry - 1.8 * (tr[1] - tr[0]) / np.linalg.norm(tr[1] - tr[0]), '#a0a0a0', lw=0.7, ms=4, z=8)
e1 = pushes[0][2]
label(ax, e1 + np.array([-2.2, -3.4]), 'other side\nstored', '#7a7a7a', ha='right', size=6.5)
tr1 = pushes[0][4]
k = next(i for i, p in enumerate(tr1) if free(p))
mx = tr1[k][0] - 1.2
ax.annotate('', xy=(mx, tr1[-1][1]), xytext=(mx, tr1[k][1]),
            arrowprops=dict(arrowstyle='|-|', lw=0.5, color=C_PUSH, shrinkA=0, shrinkB=0,
                            mutation_scale=1.5), zorder=11)
label(ax, (mx - 0.6, 0.5 * (tr1[k][1] + tr1[-1][1])), '$m$', C_PUSH, ha='right', size=8.5)
endpoints(ax)

# (c) post-processing
ax = axes[2]
base(ax, '(c) simplify and smooth')
ax.plot([p[0] for p in lazy], [p[1] for p in lazy], '-', color=C_PATH, lw=0.9, alpha=0.3, zorder=4)
ax.plot(final[:, 0], final[:, 1], '-', color=C_PATH, lw=1.6, zorder=6)
endpoints(ax)

handles = [
    Patch(fc=C_OBST, ec='none', label='obstacle'),
    Patch(fc=C_INFL, ec='none', label='inscribed zone'),
    Line2D([], [], ls='--', color=C_LINE, lw=1.0, label='straight line'),
    Line2D([], [], color=C_FORCE, lw=1.4, label='repulsion'),
    Line2D([], [], color=C_PUSH, lw=0.9, marker='s', ms=3.2, label='push and locked waypoint'),
    Line2D([], [], color=C_PATH, lw=1.5, label='path'),
]
fig.subplots_adjust(left=0.004, right=0.996, top=0.996, bottom=0.10)
fig.canvas.draw()
y0 = axes[0].get_position().y0
fig.legend(handles=handles, loc='upper center', ncol=6, frameon=False, fontsize=7.5,
           bbox_to_anchor=(0.5, y0 - 0.005), handlelength=1.8, columnspacing=1.5, handletextpad=0.5)
for ext, kw in (('pdf', {}), ('png', {'dpi': 300})):
    fig.savefig(os.path.join(here, 'method_idea.' + ext), bbox_inches='tight', pad_inches=0.015, **kw)
for p_ in pushes:
    print('entry', p_[2], '-> waypoint', np.round(p_[3], 2), 'side', p_[5])
print('F', np.round(F1, 3), 'n', n1 * side1)
