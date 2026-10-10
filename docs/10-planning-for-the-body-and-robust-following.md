# 10 — Planning for the body & robust corner following

Why a *centered* A\* path still let the robot hit walls, then get stuck — and the two
fixes: **C-space inflation** (plan for the body, not a point) and a **robust Pure
Pursuit** (adaptive lookahead + turn-in-place + curvature slowdown).

---

## 0. The symptom → the diagnosis

Path looked centered, yet the robot clipped corners; after a bump it wandered and
re-collided. Three layers:

1. **Planner plans for a POINT** on a raw, walls-only grid. The clearance cost is a
   *soft bias*, not a guarantee the robot's *body* fits.
2. **Pure Pursuit cuts corners.** It chases a carrot `Ld` ahead and knows nothing
   about walls; at a 90° bend the carrot is *around* the corner, so the arc to it
   slices the inside wall. Cut error grows with `Ld`.
3. **No localization to catch drift.** A bump slips the wheels → odometry drifts →
   the odom-frame `/plan` no longer lines up with the real maze → steer into wall →
   slip more → loop. Our `map→odom` is a *static* constant with nothing correcting it.

Measurement was the key: the planned path already kept **0.70 m** from walls (robot is
0.18 m), so the *path* wasn't the problem — the *controller* was. Fix (2), and (3)'s
loop stops starting.

---

## Part A — C-space inflation (plan for the body)

### Theory
Plan in the robot's **configuration space**. For a circular robot of radius `r`, the
C-space obstacles are the workspace walls **grown (Minkowski sum) by `r`**. Then a
**point** at a C-space-free cell guarantees the *body* is collision-free in the world.
Concretely: mark every free cell within `r` (+ margin) of a wall as occupied, and plan
on that inflated grid. This is exactly a **Nav2 costmap inflation layer**.

### Hard vs soft — we use both
- **Inflation = hard.** A guarantee: the path physically cannot pass where the body
  wouldn't fit; keeps `r+margin` from *every* wall, corners included.
- **Clearance cost = soft.** A preference: centre within whatever free space remains.

### Implementation
`astar.inflate_obstacles(grid, radius_cells)` (BFS distance transform → threshold).
The node computes `radius_cells = ceil((robot_radius + safety_margin)/res)` and inflates
in `_on_map` (once), then runs the soft clearance cost on the inflated grid.
Params: `robot_radius` (0.18 m), `safety_margin` (0.10 m) → 3 cells (0.3 m) at 0.1 m.
Verified: every path cell stays ≥ 0.3 m from any real wall (0.12 m body margin).

> Gotcha: a 2D Nav Goal clicked *against* a wall now falls inside the inflation →
> "no path". Click in open corridor (or add nearest-free snapping later).

---

## Part B — Robust Pure Pursuit (fix the corner-cutting)

The follower is differential-drive — it can **pivot in place**, the right tool for
sharp corners. Three upgrades over textbook fixed-lookahead pursuit:

1. **Adaptive lookahead:** `Ld = clamp(gain·|v|, Ld_min, Ld_max)`. Fast on straights →
   long `Ld` (smooth, stable); slow into a corner → short `Ld` (tight tracking). A
   *single fixed* `Ld` can't serve both — that's the whole trade-off:

   | | short `Ld` | long `Ld` |
   |---|---|---|
   | corners | tight ✓ | cuts, hits wall ✗ |
   | straights | wobbles ✗ | smooth ✓ |

2. **Turn-in-place:** if the carrot is more than `turn_in_place_angle` (~40°) off
   heading, set `v=0` and pivot toward it — don't arc through the corner.
3. **Curvature slowdown:** `v = max(min_linear, max_linear / (1 + slow_gain·|κ|))` —
   full speed when straight, slower as the turn sharpens. (This *is* the deferred
   "curvature-regulated speed / comfort" idea.)

> **Bootstrap-stall trap (learned the hard way).** Coupling `Ld` to *measured* speed
> plus an aggressive curvature slowdown can deadlock: at standstill `Ld = ld_min`; if
> `ld_min` is tiny the curvature to any offset carrot is huge (∝1/Ld), which crushes
> `v` toward 0; the robot never moves, so `v_meas≈0`, so `Ld` stays tiny — it only
> spins. Two guards: keep **`ld_min` sane (~0.5 m)**, and a **`min_linear` anti-stall
> floor** (~0.12 m/s) so the drive branch always makes real progress, building the
> speed that grows `Ld`. First try (`ld_min=0.25`, `slow_gain=1.5`, no floor) spun in
> place; shipped (`ld_min=0.5`, `slow_gain=0.8`, floor `0.12`) drives.

> **Sparse-path carrot trap (the near-goal spin).** The A\* path is *pruned* to
> endpoints on straights, so a **vertex-only** lookahead ("first path vertex ≥ Ld
> from the closest vertex") can return the closest vertex *behind* the robot — it
> then turns around to chase it and **spins in place** (classic "carrot flips
> behind", re-triggered by pruning; it bit us ~1 m from the goal). Fix: make
> `lookahead_point` **interpolate a carrot at arc-length Ld ahead of the robot's
> projection onto the path**, walking forward segment-by-segment. The carrot is then
> always ahead, dense or sparse, and it returns the goal once the path ends within Ld.

### Design: framework-free core
The control law lives in `vto_control/pursuit_core.py` — **no ROS imports** — so it's
host-testable and the node is a thin adapter (same split as `astar.py`). The node reads
pose + measured speed from `/odom`, calls `adaptive_lookahead`, `lookahead_point`,
`velocity_command`, and publishes `/cmd_vel`.

Verified in a unicycle sim on an L-path: max deviation **0.15 m → 0.08 m** (and unit
tests for pivot, slowdown, adaptive Ld). Idealised sim understates the real gain —
with actual inertia and true 90° corners the pivot helps more.

---

## Part C — Why #3 (can't recover) is really a Phase-2 problem

Preventing collisions stops the drift *loop*, but *any* wheel slip still drifts odom,
and our static `map→odom` can't correct it. The real fix is **localization**
(robot_localization EKF / AMCL) publishing a live, drift-corrected `map→odom`, plus
**replanning** when the robot falls off the path. That's Phase 2 — deliberately out of
scope here; today we lean on "don't collide."

---

## Interview angle

- **Q: Why inflate the map instead of just planning the shortest path?** The planner
  treats the robot as a point; inflating obstacles by the robot radius (C-space /
  Minkowski) makes a point-plan body-safe. Standard costmap inflation.
- **Q: Soft clearance cost vs hard inflation — why both?** Inflation *guarantees*
  feasibility (lethal band = robot radius); the soft cost *centres* within the
  remaining space. Guarantee + preference.
- **Q: Why does Pure Pursuit cut corners, and how to fix it?** The carrot sits `Ld`
  ahead; at a bend the chord to it cuts the corner (error ∝ `Ld`). Fixes: shrink/adapt
  `Ld`, slow on curvature, and for a robot that can rotate in place, pivot at large
  heading error.
- **Q: Trade-off of the lookahead distance?** Short = tight tracking but oscillates on
  straights; long = smooth but cuts corners. Adaptive (∝ speed) resolves it.
- **Q: Robot bumps a wall and then can't recover — root cause?** Odometry drift with no
  localization to correct `map→odom`; the plan de-aligns from the world. Need
  localization (EKF/AMCL) + replanning.
