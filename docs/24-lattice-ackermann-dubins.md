# 24 — Ackermann state lattice, step 1: Dubins connection primitives

Doc 22 built a state lattice for **M1 (diff-drive)**. Doc 20 §1 marks Hybrid A\* /
lattice as a **"Direct fit"** for **M2 (car-like)** — the textbook use case. This
doc starts that build. It is **step 1 of 3**:

1. **(this doc)** `dubins.py` — the bounded-curvature *edge* primitive: given two
   oriented poses, the shortest car-drivable curve between them.
2. `lattice_ackermann.py` — enumerate candidate lattice endpoints, connect each
   with a Dubins curve, prune to a minimal **control set**, then search (reuse the
   `lattice()` machinery from doc 22).
3. `lattice_ackermann_planner_node.py` — ROS adapter, drop-in with the A\* and
   diff-drive-lattice nodes.

Design + code + host tests for step 1 only. No ROS.

---

## 1. General idea / theoretical aspect

### What a lattice edge has to be, for a car

In doc 22 (diff-drive) an edge was "pick an arc of a radius I like, simulate it,
**snap** the endpoint heading to the nearest bin, and let an in-place pivot mop up
whatever heading error the snap left"
([lattice.py:82-86](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L82-L86)).

A car-like robot (M2, [doc 20 §0](20-trajectory-generation-roadmap.md)) breaks that
in two ways:

- **No pivot.** Heading only changes while translating. There is no primitive that
  absorbs a leftover heading mismatch, so an edge's endpoint pose must land
  **exactly** on a lattice node `(Δx, Δy, Δθ_bin)` — not "close, snap it".
- **Bounded curvature.** Steering angle `φ` is limited, so
  `|κ| ≤ κ_max = tan(φ_max) / L = 1 / R_min`. An edge can't be an arc of any radius
  — only radius `≥ R_min`.

So a car-like lattice edge is the answer to a **boundary-value problem (BVP)**:

> Find the shortest path from pose `(0, 0, 0)` to pose `(x, y, θ)` such that
> curvature never exceeds `κ_max`.

### Dubins' result

Lester Dubins (1957) solved exactly this BVP for a car that **only moves forward**
at constant speed with a minimum turning radius. His result: the shortest such path
is **always** a sequence of at most three segments, each one of

- **`L`** — arc left at radius `R_min` (`κ = +κ_max`),
- **`R`** — arc right at radius `R_min` (`κ = -κ_max`),
- **`S`** — straight (`κ = 0`),

and only **6 combinations** ever occur:

```
    C S C  type :  LSL   RSR   LSR   RSL      ("turn, cruise, turn")
    C C C  type :  RLR   LRL                  ("turn, counter-turn, turn" — tight goals)
```

Each of the 6 "words" has a **closed-form** solution for its three segment lengths
`(t, p, q)` as a function of the normalized goal pose — no iteration. Compute all 6,
discard the infeasible ones, take the shortest. That is the whole algorithm.

### Why "bang-bang in steering"

The deeper reason there are only 3 segment types comes from **Pontryagin's Maximum
Principle**: for a shortest-path problem with a bounded control (steering), the
optimal control sits at an extreme of its range almost everywhere — full left, full
right, or (on a singular arc) centered. Steering is never "halfway". That collapses
an infinite-dimensional path search into choosing among a handful of
**bang-bang switching sequences**, and Dubins showed ≤ 3 switches suffice.

### Where it sits in the pipeline

Same slot as doc 22: **global generation**. `dubins.py` is a geometry library;
`lattice_ackermann.py` (step 2) will call it hundreds of times at startup to build
the primitive table, then the search is the same A\* over `(row, col, heading_idx)`
as doc 22.

---

## 2. Method selection — why Dubins here, and why not the alternatives

**Chosen:** forward-only **Dubins shortest path**, at fixed radius `R_min`, as the
lattice edge-connection primitive.

### Why not just reuse the diff-drive `lattice.py` with a big radius?

Its arcs are *"chosen, not solved for"* and the endpoint heading is *snapped*
([lattice.py:82-86](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L82-L86)).
For diff-drive the snap error is harmless — a pivot fixes it. For a car there is no
pivot, so snapped primitives **don't compose into a drivable path**: every edge
join carries a small heading discontinuity the robot can't execute. A car lattice
*must* solve the exact BVP per edge. That's what Dubins gives.

### Why not Reeds–Shepp (allow reverse)?

Reeds–Shepp (1990) generalizes Dubins to a car that can also **reverse**, adding
cusps. It has **~48 words** instead of 6, produces strictly shorter or equal paths,
and is **required** for genuinely tight maneuvering — parallel parking, 3-point
turns in a corridor narrower than the turning circle.

- **Deferred, not rejected.** Forward-only is the right first slice: it gets the
  full pipeline (enumerate → prune → search → node → benchmark) working end to end
  with 6 words to verify instead of 48. Reeds–Shepp then drops in behind the same
  `DubinsPath`-shaped interface (`.word`, `.segments`, `.length`, `sample_*`)
  without touching steps 2–3.
- **Honest caveat for our maze:** corridors are 1.5 m
  ([doc 06](06-maze-environment.md)). A car with `R_min` much above ~0.75 m
  *cannot* turn a corridor junction forward-only and will need reverse. So step 2
  will report when no forward-only primitive set can solve the maze, and that's the
  trigger to bring Reeds–Shepp forward. Building Dubins first is still correct —
  Reeds–Shepp *contains* the Dubins words as its no-reversal subset, so none of
  this work is thrown away.

### Why not polynomial spirals / clothoids (Pivtoraiko & Kelly's "state lattice" proper)?

The "correct" lattice primitive is **curvature-continuous**: curvature varies
linearly along arc length (a clothoid), so steering *rate* stays finite and the
car doesn't have to snap the wheel instantly.

- Each edge is then a **nonlinear BVP** — solve for the 3 spiral parameters
  `(a, b, s_f)` with Newton's method per candidate endpoint. More code, more failure
  modes (non-convergence), harder to unit-test to a known answer.
- Dubins is **curvature-bounded but curvature-discontinuous**: it commands an
  instantaneous steering jump at each `L`↔`S`↔`R` switch. Acceptable here because
  (a) a downstream smoother or (b) the tracker's own steering-rate limit or (c)
  MPC/MPPI re-optimizing over the reference all absorb the jump — the lattice path
  is a *guide*, not the executed trajectory. Doc 13's MPC already re-solves every
  tick.
- Spirals remain the natural upgrade if the discontinuity turns out to matter in
  practice. Same interface swap as Reeds–Shepp.

### Why not Hybrid A\* fixed-input primitives (Dolgov et al.)?

Hybrid A\* doesn't connect to exact lattice nodes at all — it applies a few fixed
steering inputs for a fixed time, lets endpoints land **anywhere**, and uses the
grid cell only for the closed set (keeping the continuous pose). It avoids the BVP
entirely.

- But then the search state is **continuous**, and doc 22's clean machinery —
  exact `(row, col, heading_idx)` keys, "generate primitives once at heading 0 and
  **rotate** them to every other heading"
  ([lattice.py:107-122](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L107-L122))
  — no longer applies, because a rotated fixed-input primitive doesn't land on a
  node.
- Connecting to exact nodes with Dubins keeps *all* of doc 22's search code reusable
  for step 2. The cost is solving the BVP — which is closed-form, so it's cheap.
- (Hybrid A\* is still a reasonable alternative overall; the deciding factor is
  code reuse with doc 22, plus that an exact lattice gives a cleaner story for the
  benchmark comparison against the diff-drive lattice.)

### Summary

| Method | Reverse? | Curvature-continuous? | Per-edge cost | Lands on lattice nodes? |
|---|---|---|---|---|
| Diff-drive `lattice.py` arcs | pivot instead | no (and snapped) | closed-form | only after snap+pivot |
| **Dubins (chosen)** | no | no (bounded) | **closed-form, 6 words** | **yes, exact** |
| Reeds–Shepp | yes | no (bounded) | closed-form, ~48 words | yes, exact |
| Clothoid spirals | optional | **yes** | Newton BVP solve | yes, exact |
| Hybrid A\* fixed-input | optional | no | forward sim | **no** |

---

## 3. Coding — design of `dubins.py`

Framework-free, host-testable, same spirit as `astar.py` / `lattice.py`
([lattice.py:1-30](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L1-L30)).
Works in a **neutral length unit**; step 2 will call it in **cell units** with
`radius = R_min / map_resolution`, exactly as `lattice.py` is "entirely CELL-based".

### `mod2pi(theta)` — wrap to `[0, 2π)`

Every word formula produces angles that must be normalized before they mean "arc
sweep". One helper, used everywhere.

### Six word functions `_lsl … _lrl` — private, pure

Signature: `(alpha, beta, d) -> (t, p, q) | None`, where `alpha`, `beta` are the
start/goal headings expressed relative to the chord direction and `d` is the
straight-line distance **normalized by radius**. Returns the three segment lengths
in normalized units, or `None` if that word's geometry is infeasible for this
`(alpha, beta, d)`.

- **Why six separate functions, not one branchy function:** each is a distinct
  algebraic identity from the literature (Shkel & Lumelsky 2001). Kept separate,
  each maps 1:1 to a named case and is unit-testable against a hand-worked example.
  A single function with six branches would be one long unverifiable block.
- **Why pure `(alpha, beta, d)`:** these three numbers are the *complete*
  normalized description of a Dubins query — everything else (translation,
  start-heading rotation, radius scaling) is stripped by the caller first. This is
  the standard normalization and makes the words reusable verbatim if we later add
  Reeds–Shepp (which builds on the same normalized frame).

### `DubinsPath` — a small value object

Fields: `start`, `goal`, `radius`, `word` (`"LSR"`), `seg_norm` (`(t, p, q)`),
and derived `length`.

- **Why a class, not a tuple:** it carries a *derived* quantity — real arc length
  `= (t + p + q) · radius` (turn segments' normalized length is radians, so
  `radians · radius` is arc length; straight segment's is already `distance /
  radius`, so `· radius` recovers distance — the same `· radius` for all three,
  which is why the normalized form is chosen). Bundling the fields that
  `sample_dubins` needs together, with `length` computed once, is cleaner than
  passing a 5-tuple around. `lattice.py` used plain dicts for primitives; this one
  has actual behavior, so a class earns its place.

### `dubins_shortest_path(start, goal, radius) -> DubinsPath`

1. Express `goal` in a frame **translated to `start` and rotated so start heading
   = 0** (pure trig, no allocation).
2. Form `d = |Δ| / radius`, `alpha`, `beta`.
3. Call all six words, keep the feasible ones, return the `DubinsPath` for the
   minimum `t + p + q`.

O(1) — six closed-form evaluations. Dubins' theorem guarantees at least one word is
always feasible for `radius > 0`, so the result is never `None` (we raise if that
invariant is somehow violated, rather than returning `None`, because step 2 relies
on it).

### `sample_dubins(path, step) -> [(x, y, yaw)]`

Walk the three segments; within each, integrate the **closed-form** pose evolution
(`L`/`R` are circular-arc updates, `S` is linear) at spacing `≤ step`, carrying the
running pose from one segment into the next. Endpoints included.

- **Why closed-form, not numeric integration:** the arc update
  `x += R(sin(ψ+u) − sin ψ)`, `y += R(cos ψ − cos(ψ+u))`, `ψ += u` is exact for a
  constant-curvature arc — no integrator error, and it's what makes
  "sample endpoint == goal" a valid test to machine precision.
- **Used later by step 2** for exactly the two jobs `lattice.py` already has:
  collision-checking a primitive against the grid
  (`_primitive_is_clear`, [lattice.py:135-142](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L135-L142))
  and densifying the final path so the tracker doesn't cut curved edges
  (`densify_path`, [lattice.py:228-249](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L228-L249)).

### What is **not** in this file (later steps)

- Enumerating candidate lattice endpoints and pruning to a minimal control set
  (step 2).
- The search — reuses `lattice()` from doc 22 with the new primitive table.
- Any `(row, col)` / occupancy-grid contact — step 2 wraps this in cell units.
- The ROS node (step 3).

### Complexity

`dubins_shortest_path`: O(1). Step 2 calls it once per candidate endpoint at
startup (a few hundred), then never again — the primitive table is precomputed and
reused across the whole search, same as doc 22.

---

## Interview angle

- **Q: Why is the shortest forward-only bounded-curvature path always one of just 6
  types?** Pontryagin's Maximum Principle forces the steering control to be
  bang-bang — full left, full right, or straight — almost everywhere, and Dubins
  proved at most 3 such segments suffice. The 3-segment bang-bang sequences that
  can be optimal reduce to `CSC` (LSL, RSR, LSR, RSL) and `CCC` (RLR, LRL).
- **Q: Dubins vs Reeds–Shepp?** Reeds–Shepp allows reverse (cusps): ~48 words,
  strictly shorter-or-equal paths, and *necessary* for maneuvering in spaces
  tighter than the turning circle (parking, 3-point turns). Dubins is the
  forward-only subset — simpler, fine when there's room to loop around. Reeds–Shepp
  contains the Dubins words, so a Dubins implementation is a stepping stone, not
  wasted work.
- **Q: What's the "non-holonomic heuristic" in Hybrid A\*, and how does Dubins
  relate?** It's the cost of the Dubins (or Reeds–Shepp) path to the goal
  *ignoring obstacles*. It respects the kinematic limit a straight-line/Euclidean
  heuristic ignores, so it's a much tighter admissible estimate near the goal where
  heading alignment dominates the remaining cost. Hybrid A\* takes the max of this
  and an obstacle-aware holonomic heuristic.
- **Q: Dubins commands an instantaneous steering change at each segment switch —
  isn't that unexecutable?** Yes, curvature is discontinuous. Fixes: clothoid/spiral
  primitives (curvature varies linearly, steering rate finite), post-hoc smoothing,
  or letting a receding-horizon controller (MPC/MPPI) re-optimize over the Dubins
  path as a reference rather than tracking it literally.
- **Q: Why doesn't the diff-drive lattice need any of this?** A diff-drive robot
  can pivot in place and has unbounded curvature, so an edge can be "any arc I
  like, snap the endpoint, pivot to fix the residual". A car has neither escape
  hatch, so every edge must be the *exact* solution of a curvature-bounded
  boundary-value problem between two oriented poses — which is what a Dubins curve
  is.
- **Q: Why normalize the query to `(alpha, beta, d)` before solving?** A Dubins
  problem has only 3 intrinsic degrees of freedom once you quotient out the start
  pose (SE(2), 3 DOF) and scale by radius. `(alpha, beta, d)` is that minimal
  description, so the six closed-form identities can be written once and reused
  (including later for Reeds–Shepp).

---

## Check-in

Two things to confirm before I write step 2 (`lattice_ackermann.py`):

1. **Forward-only Dubins now, Reeds–Shepp deferred** — agreed? (Step 2 will tell us
   empirically whether the maze forces reverse.)
2. Proceed straight to **step 2 (endpoint enumeration + control-set pruning)** after
   you've read this, or do you want to sit with step 1's code first?
