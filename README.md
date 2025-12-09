# Lazy Coulomb Planner

## Overview

The **Lazy Coulomb Planner** is a reactive path planning algorithm inspired by electrostatic repulsion principles from physics. The algorithm treats obstacles as negatively charged bodies and path intersection points as like-charged particles, causing them to be repelled away from obstacles until a collision-free path is found.

The "lazy" aspect comes from its greedy approach: it starts with the simplest possible solution (a straight line) and only does work when absolutely necessary—when the path intersects an obstacle.

---

## Table of Contents

1. [Similar Methods](#similar-methods)
2. [What Makes This Unique](#what-makes-this-unique)
3. [Applications](#applications)
4. [Limitations](#limitations)
5. [Methodology](#methodology)
6. [Technical Description](#technical-description)
7. [Parameters](#parameters)
8. [References](#references)

---

## Similar Methods

### 1. Artificial Potential Field (APF)
The most closely related approach. APF creates an attractive potential at the goal and repulsive potentials around obstacles. The robot follows the gradient descent of the combined potential field.

| Aspect | APF | Lazy Coulomb Planner |
|--------|-----|---------------------|
| Planning | Continuous field evaluation | Discrete intersection points |
| Goal handling | Attractive force toward goal | Direct line to goal, no attraction |
| Computation | Every point in space | Only at obstacle boundaries |
| Local minima | Prone to getting stuck | Uses perpendicular escape |

### 2. Rapidly-exploring Random Trees (RRT)
A sampling-based algorithm that builds a tree of random configurations to explore the space.

- **Pros**: Works well in high-dimensional spaces, handles complex constraints
- **Cons**: Paths are jagged, requires post-processing for smoothness
- **Difference**: RRT explores randomly; Lazy Coulomb is deterministic and reactive

### 3. A* Algorithm
A graph-based search algorithm that finds the optimal path using heuristics.

- **Pros**: Guarantees shortest path, complete algorithm
- **Cons**: Requires discretized space, computationally expensive for large spaces
- **Difference**: A* plans globally; Lazy Coulomb reacts locally

### 4. Visibility Graph
Connects all vertices of obstacles that can "see" each other, then searches for shortest path.

- **Pros**: Optimal paths in 2D polygonal environments
- **Cons**: Expensive to compute, doesn't handle dynamic obstacles
- **Difference**: Visibility graph is global; Lazy Coulomb is incremental

### 5. Bug Algorithms (Bug0, Bug1, Bug2)
Simple reactive algorithms where the robot follows obstacles boundaries when encountered.

- **Pros**: Minimal memory, works with unknown environments
- **Cons**: Inefficient paths, can get stuck in certain configurations
- **Difference**: Bug algorithms follow boundaries; Lazy Coulomb pushes away perpendicularly

### 6. Vector Field Histogram (VFH)
Creates a polar histogram of obstacle density and steers toward gaps.

- **Pros**: Real-time capable, handles sensor noise
- **Cons**: Local method, can oscillate in narrow passages
- **Difference**: VFH uses density histograms; Lazy Coulomb uses direct repulsion

---

## What Makes This Unique

### 1. Lazy Evaluation
Unlike potential field methods that compute forces everywhere, Lazy Coulomb only computes repulsion at **actual intersection points**. No intersection = no computation.

### 2. Boundary-Centric Approach
The algorithm specifically targets **boundary crossing points** rather than evaluating the entire path. This focuses computational effort where it matters.

### 3. Perpendicular Escape Strategy
Instead of following gradient descent (which can lead to local minima), the algorithm escapes **perpendicular to the path direction**, maintaining path efficiency.

### 4. Progressive Locking
Once a point successfully escapes an obstacle, it becomes **locked** and never moves again. This prevents oscillation and guarantees progress.

### 5. Segment-Based Collision Detection
Rather than checking individual points, the algorithm verifies entire **line segments** are collision-free, ensuring no obstacle crossings are missed.

### 6. Inflation Radius Support
Built-in safety margins around obstacles ensure the path maintains a configurable minimum distance from all obstacles.

### 7. Multi-Shape Support
Handles rectangles, circles, triangles, and boundary walls with unified collision detection and repulsion calculation.

---

## Applications

### Well-Suited Applications

#### 1. Mobile Robot Navigation
- Warehouse robots navigating around shelves and equipment
- Service robots in open floor plans (hospitals, hotels, offices)
- Agricultural robots in fields with scattered obstacles

#### 2. Drone Path Planning
- UAV navigation around buildings in urban environments
- Indoor drone flight avoiding furniture and fixtures
- Aerial survey with no-fly zone avoidance

#### 3. Game AI
- NPC pathfinding in open-world games
- Real-time strategy unit movement
- Smooth character navigation around obstacles

#### 4. Autonomous Vehicles
- Parking lot navigation
- Golf carts on courses
- Low-speed autonomous shuttles

#### 5. Marine Navigation
- Ship routing around islands and shallow areas
- Underwater vehicle obstacle avoidance
- Sailboat race course optimization

#### 6. Animation and Graphics
- Camera path planning in 3D software
- Smooth transitions avoiding scene geometry
- Procedural animation paths

### Ideal Environment Characteristics
- Open spaces with scattered obstacles
- Convex or near-convex obstacles
- Low to medium obstacle density
- Real-time requirements
- Smooth paths preferred over optimal paths

---

## Limitations

### 1. Narrow Passages
**Problem**: When obstacles form narrow corridors, both perpendicular escape directions may be blocked.

**Symptom**: Point oscillates or gets stuck between parallel obstacles.

**Mitigation**: Ensure obstacle spacing exceeds 2× inflation radius, or use a different algorithm for maze-like environments.

### 2. Concave Obstacles
**Problem**: U-shaped or concave obstacles can trap the path inside.

**Symptom**: Point cannot find escape direction, infinite iterations.

**Mitigation**: Decompose concave obstacles into convex components, or increase iteration limits with random perturbation.

### 3. No Global Optimality
**Problem**: The algorithm finds *a* path, not necessarily the *shortest* path.

**Symptom**: Paths may have unnecessary detours.

**Mitigation**: Use as a local planner with global waypoints from A* or similar.

### 4. Dead Ends
**Problem**: Cannot backtrack if path leads into a dead end.

**Symptom**: Algorithm fails to find path even when one exists.

**Mitigation**: Not suitable for maze-solving; use BFS/DFS/A* instead.

### 5. High Obstacle Density
**Problem**: With many closely-spaced obstacles, finding clear perpendicular directions becomes difficult.

**Symptom**: Slow convergence, excessive iterations.

**Mitigation**: Reduce inflation radius, or switch to sampling-based methods.

### 6. Dynamic Obstacles
**Problem**: Current implementation assumes static obstacles.

**Symptom**: Locked points may become invalid if obstacles move.

**Mitigation**: Implement periodic re-planning or unlock points when obstacles change.

---

## Methodology

### Algorithm Overview

The Lazy Coulomb Planner follows a simple iterative process:

```
1. INITIALIZE: Draw straight line from Start (S) to Goal (G)
2. CHECK: Find first segment that crosses any obstacle
3. IF no crossing found → PATH COMPLETE ✓
4. INSERT: Add new point at intersection location
5. PUSH: Apply repulsion force to move point outside obstacle
6. LOCK: Mark point as fixed once outside
7. CLEANUP: Remove any unlocked intermediate points
8. GOTO Step 2
```

### Detailed Methodology

#### Phase 1: Initialization
```
Path = [Start, Goal]
LockedPoints = {Start, Goal}
```

The algorithm begins with the laziest possible assumption: maybe we can go straight there!

#### Phase 2: Segment Intersection Detection

For each consecutive pair of points (P₁, P₂) in the path:
1. Sample points along the segment at regular intervals
2. For each sample point, check if it lies inside any obstacle (including inflation zone)
3. If intersection found, record the segment index and obstacle

```
for each segment (Pᵢ, Pᵢ₊₁):
    for t in [0.1, 0.2, ..., 0.9]:
        sample = Pᵢ + t × (Pᵢ₊₁ - Pᵢ)
        for each obstacle:
            if sample inside obstacle:
                return (segment_index: i, obstacle: obstacle)
```

#### Phase 3: Point Insertion

When an intersection is found:
1. Calculate the exact entry point where segment crosses obstacle boundary
2. Insert new point at this location
3. Update locked point indices to account for insertion

```
intersection_point = find_entry_point(segment, obstacle)
path.insert(segment_index + 1, intersection_point)
shift_locked_indices(segment_index + 1)
```

#### Phase 4: Repulsion (The Core Physics)

The inserted point is pushed out using electrostatic-inspired repulsion:

1. **Calculate path direction** between locked neighbors
2. **Determine perpendicular directions** (left and right of path)
3. **Find best escape direction** that leads to clear space fastest
4. **Apply repulsion force** in that direction
5. **Repeat** until point is outside all obstacles

```
while point inside obstacle:
    path_direction = normalize(next_locked - prev_locked)
    perp_left = rotate90CCW(path_direction)
    perp_right = rotate90CW(path_direction)
    
    best_direction = find_direction_to_clear_space([perp_left, perp_right, ...])
    
    force = best_direction × REPULSION_STRENGTH
    point.position += force × step_size
```

#### Phase 5: Point Locking

Once a point successfully exits the obstacle (including inflation zone):
1. Mark point as **locked**
2. Point will never move again
3. Provides anchor for future calculations

```
if point outside obstacle:
    locked_points.add(point_index)
    point.locked = true
```

#### Phase 6: Cleanup and Iteration

After locking a point:
1. Remove any unlocked intermediate points (failed attempts)
2. Return to Phase 2 to check for remaining intersections
3. Continue until no segments cross any obstacles

---

## Technical Description

### Coordinate System

- **Grid**: 40 × 40 cells (configurable)
- **Origin**: Top-left corner (0, 0)
- **X-axis**: Increases rightward
- **Y-axis**: Increases downward

### Obstacle Representations

#### Rectangle
```
obstacle = {
    type: 'rectangle',
    x1: min_x,    // Left edge
    y1: min_y,    // Top edge
    x2: max_x,    // Right edge
    y2: max_y     // Bottom edge
}
```

#### Circle
```
obstacle = {
    type: 'circle',
    cx: center_x,
    cy: center_y,
    radius: r
}
```

#### Triangle
```
obstacle = {
    type: 'triangle',
    points: [
        {x: x1, y: y1},
        {x: x2, y: y2},
        {x: x3, y: y3}
    ]
}
```

### Collision Detection Equations

#### Rectangle (with inflation radius ρ)

A point (x, y) is inside the inflated rectangle if:

$$x_1 - \rho \leq x \leq x_2 + \rho$$

$$y_1 - \rho \leq y \leq y_2 + \rho$$

#### Circle (with inflation radius ρ)

A point (x, y) is inside the inflated circle if:

$$\sqrt{(x - c_x)^2 + (y - c_y)^2} \leq r + \rho$$

#### Triangle (with inflation radius ρ)

**Step 1**: Check if inside triangle using barycentric coordinates.

Given triangle vertices A, B, C and point P:

$$\vec{v_0} = C - A$$
$$\vec{v_1} = B - A$$
$$\vec{v_2} = P - A$$

$$d_{00} = \vec{v_0} \cdot \vec{v_0}$$
$$d_{01} = \vec{v_0} \cdot \vec{v_1}$$
$$d_{02} = \vec{v_0} \cdot \vec{v_2}$$
$$d_{11} = \vec{v_1} \cdot \vec{v_1}$$
$$d_{12} = \vec{v_1} \cdot \vec{v_2}$$

$$\text{denom} = d_{00} \cdot d_{11} - d_{01} \cdot d_{01}$$

$$u = \frac{d_{11} \cdot d_{02} - d_{01} \cdot d_{12}}{\text{denom}}$$

$$v = \frac{d_{00} \cdot d_{12} - d_{01} \cdot d_{02}}{\text{denom}}$$

Point is inside if: $u \geq 0$ AND $v \geq 0$ AND $u + v \leq 1$

**Step 2**: If outside, check distance to edges for inflation zone.

Distance from point P to line segment AB:

$$d = \frac{|(B-A) \times (A-P)|}{|B-A|}$$

If $d \leq \rho$ and projection falls within segment, point is in inflation zone.

#### Wall/Boundary (with inflation radius ρ)

A point (x, y) is inside the wall inflation zone if:

$$x < \rho \quad \text{OR} \quad x > (W - 1) - \rho$$
$$y < \rho \quad \text{OR} \quad y > (H - 1) - \rho$$

Where W and H are grid width and height.

### Repulsion Force Calculation

#### Path Direction Vector

Given previous locked point $P_{prev}$ and next locked point $P_{next}$:

$$\vec{d}_{path} = \frac{P_{next} - P_{prev}}{|P_{next} - P_{prev}|}$$

#### Perpendicular Directions

Left perpendicular (90° counter-clockwise):
$$\vec{d}_{left} = (-d_{path,y}, \quad d_{path,x})$$

Right perpendicular (90° clockwise):
$$\vec{d}_{right} = (d_{path,y}, \quad -d_{path,x})$$

#### Direction Selection

For each candidate direction $\vec{d}_i$, find distance to clear space:

$$\text{clearDist}_i = \min\{t > 0 : P + t \cdot \vec{d}_i \text{ is outside all obstacles}\}$$

Select direction with minimum clear distance:

$$\vec{d}_{best} = \arg\min_{\vec{d}_i} \text{clearDist}_i$$

#### Force Application

$$\vec{F} = \vec{d}_{best} \times k_{repulsion}$$

$$P_{new} = P_{old} + \vec{F} \times \Delta t$$

Where:
- $k_{repulsion}$ = Repulsion strength constant (default: 2.5)
- $\Delta t$ = Step size (default: 0.5)

### Convergence Criteria

The algorithm terminates when:

1. **Success**: No segment intersects any obstacle
   $$\forall i : \text{segment}(P_i, P_{i+1}) \cap \text{Obstacles} = \emptyset$$

2. **Failure**: Maximum iterations exceeded
   $$\text{iterations} > \text{MAX\_ITERATIONS}$$

### Segment Intersection Test

A segment from $P_1$ to $P_2$ intersects an obstacle if any sampled point is inside:

$$\exists t \in (0, 1) : P_1 + t(P_2 - P_1) \in \text{Obstacle}$$

Sampling resolution:
$$n_{samples} = \max(\lceil|x_2 - x_1|\rceil, \lceil|y_2 - y_1|\rceil, 10)$$

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `GRID_SIZE` | 40 | Grid dimensions (cells) |
| `CELL_SIZE` | 15 | Pixel size per cell |
| `REPULSION_STRENGTH` | 2.5 | Force magnitude multiplier |
| `REPULSION_RADIUS` | 6 | Maximum repulsion influence distance |
| `PERTURBATION_STRENGTH` | 1.5 | Random perturbation when stuck |
| `FORCE_BALANCE_THRESHOLD` | 0.3 | Minimum force before perturbation |
| `MAX_ITERATIONS` | 500 | Maximum algorithm iterations |
| `inflationRadius` | 1.5 | Safety margin around obstacles |

### Parameter Tuning Guide

**Increase `REPULSION_STRENGTH` when:**
- Points are moving too slowly
- Algorithm takes too many iterations

**Decrease `REPULSION_STRENGTH` when:**
- Points are overshooting clear space
- Path has unnecessary oscillations

**Increase `inflationRadius` when:**
- Larger safety margins needed
- Working with imprecise actuators

**Decrease `inflationRadius` when:**
- Navigating tight spaces
- High precision available

---

## References

1. **Khatib, O.** (1986). "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots." *The International Journal of Robotics Research*, 5(1), 90-98.
   - Original Artificial Potential Field method

2. **Coulomb, C.A.** (1785). "Premier Mémoire sur l'Électricité et le Magnétisme."
   - Foundation of electrostatic force laws

3. **LaValle, S.M.** (1998). "Rapidly-Exploring Random Trees: A New Tool for Path Planning."
   - RRT algorithm reference

4. **Hart, P.E., Nilsson, N.J., Raphael, B.** (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100-107.
   - A* algorithm original paper

5. **Borenstein, J., Koren, Y.** (1991). "The Vector Field Histogram - Fast Obstacle Avoidance for Mobile Robots." *IEEE Transactions on Robotics and Automation*, 7(3), 278-288.
   - VFH method reference

---

## License

MIT License - Feel free to use, modify, and distribute.

---

## Author

Developed as an experimental path planning visualization tool.

*"Why plan ahead when you can just push through?"* - The Lazy Coulomb Philosophy
