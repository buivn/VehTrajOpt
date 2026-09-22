# 30 — Lattice goal docking: closing the tolerance gap

## Background: what broke

In the `lattice_pursuit_smoke` run (5 legs, 1 repeat — see the benchmark
labeling convention in `bench.launch.py`'s docstring: `*_smoke` is a quick
sanity pass, `*_confirm` is the real evaluation scale), 3/5 legs timed out at
150s instead of reaching goal. Inspecting the tick log showed the robot
fully **stopped** (`v=0, w=0`) 0.28–0.38 m short of the requested goal, not
oscillating or stuck against an obstacle — pure pursuit's own "close enough,
stop" logic had already fired.

## 1. General idea / theory

A **state lattice** plans over a *discrete* graph: states are
`(row, col, heading_bin)`, and edges are a small fixed library of
precomputed motion primitives (straight run, two arc curvatures each way,
two in-place pivots — see `docs/22`). This graph only contains states
reachable by *exact combinations* of those primitives from the start. An
arbitrary continuous goal `(gx, gy)` almost never coincides with a graph
vertex, so the search can't do an exact-match goal test the way A* does on a
plain occupancy grid (A* accepts the exact goal *cell*, and a grid cell is
tiny — 0.1 m here — relative to the primitive step size of 3 cells).

`lattice.py`'s `reached()` (`lattice.py:184-185`) therefore accepts *any*
state within a **tolerance radius** (`goal_tolerance_cells`, default 1.5
cells = 0.15 m) of the goal — standard practice for lattice planners
(Pivtoraiko & Kelly-style implementations do the same). That's a deliberate,
correct relaxation of the *search*. The bug was downstream: nothing ever
closed that residual 0.15 m gap back to the literal goal before handing the
path to the tracker.

Compounding it: pure pursuit has its **own** independent stop condition —
"stop once I'm within 0.25 m of the path's *last point*" (`pure_pursuit_node.py:98-104`).
That check is relative to whatever the planner published as the last
waypoint, not the true goal. So the robot's final resting distance from the
true goal was *lattice's own slack (≤0.15 m) plus pursuit's stop slack
(≤0.25 m), stacked* — enough to blow past the benchmark's separate 0.25 m
`goal_tolerance` in 3/5 legs. A* never has this problem because its last
waypoint already *is* (almost exactly) the true goal, so only one slack term
applies, not two.

Pipeline placement: this is purely a **planning → tracking handoff**
correctness issue — the `/plan` (`nav_msgs/Path`) contract between the
global planner and the controller. No change to search, control, or
localization.

## 2. Method selection

Three ways to close the gap, in the order I considered them:

- **(a) Tighten `goal_tolerance_cells` in the search.** Rejected: it doesn't
  fix anything, it just gambles on the gap being smaller. The gap is
  *structural* (primitive endpoints are discrete), not a tuning artifact —
  shrinking the tolerance risks the search finding **no** accepting state at
  all near some goals, trading "sometimes 0.3 m short" for "sometimes no
  path found."
- **(b) Loosen the benchmark's/pursuit's tolerance to match.** Rejected:
  this hides the mismatch instead of fixing it, and it breaks apples-to-apples
  comparison against A* (which doesn't need the slack) — exactly the kind of
  fix that looks green in a summary CSV but teaches nothing about why lattice
  planning has this property.
- **(c) Append a final straight "docking" segment from the search's
  accepted near-goal state to the literal requested goal.** This is what
  production lattice planners actually do: the lattice graph gets you
  *close* efficiently while respecting the robot's motion constraints, and a
  short terminal segment (well under one primitive step) closes the last
  few centimeters. Chosen because it fixes the actual defect — the
  planner's output no longer over-promises precision it can't structurally
  deliver — rather than papering over it with a looser number somewhere else.

Why this doesn't need to be kinematically fancy (e.g. a Dubins arc for
consistency with the Ackermann variant): the diff-drive tracker (pure
pursuit) already treats `/plan` as a general polyline of waypoints, not a
trajectory with hard curvature constraints — it re-plans the arc to each
lookahead point every tick. A 0.15 m straight hop is negligible next to the
0.5–1.0 m lookahead window (`lookahead_min`/`lookahead_max`), so it's tracked
exactly like any other short path segment, no special-casing needed. (This
would **not** carry over unchanged to the Ackermann/Dubins lattice — a
car-like model can't take an arbitrary straight hop at the end; that variant
would need a short Dubins segment instead, a note for whenever
`lattice_ackermann`'s ROS node — step 3, not started — is built.)

## 3. Coding — design

One function touched: `LatticePlanner._publish_path()` in
`lattice_planner_node.py`. No new class or data structure — this is a
handoff-format fix, not a new responsibility, so extending the existing
method (adding one optional parameter) is the right scope; a new class
would be over-engineering for "conditionally append one more point to a
list."

- **Where, not in `lattice.py`**: `lattice.py` is deliberately unit-agnostic
  (cells only, no ROS, no meters — see its module docstring, same ethos as
  `astar.py`). The *continuous* requested goal `(gx, gy)` in meters only
  exists in the ROS node (`_on_goal`, already has it at `lattice_planner_node.py:136`).
  Mixing "snap to the literal continuous goal" into the cell-based search
  core would break that abstraction boundary for no benefit — the node is
  already exactly where cell↔world conversion happens (`_world_from_cell`).
- **Signature**: `_publish_path(self, cells, exact_goal=None)`. `exact_goal`
  defaults to `None` so the method's existing behavior (used nowhere else
  yet, but keeps the function self-contained/testable) is unchanged if
  omitted.
- **Logic**: convert `cells` to `PoseStamped`s as before, then — if
  `exact_goal` is given and farther than 1 mm from the path's current last
  point (avoids a zero-length duplicate waypoint when the search happened to
  land exactly on the goal) — append one more `PoseStamped` at the literal
  goal.
- **No new collision check on the docking hop.** `lattice()` already
  requires the goal *cell* itself to be `is_free` (`lattice.py:163`), and the
  hop is at most `goal_tolerance_cells` (1.5 cells = 0.15 m) inside an
  already-inflated-free region of a 1.5 m-wide corridor — an order of
  magnitude of slack. Adding a `_primitive_is_clear`-style sample check for
  a scenario that can't realistically occur at this maze's scale would be
  validation for a case that doesn't happen, not defensive engineering
  (flagging this explicitly since it's a place I chose *not* to add a check
  that exists elsewhere in this file for arcs — the difference is arcs can
  clip a wall mid-curve on a *3-cell* primitive; this hop is 10x shorter and
  dead straight).

Complexity/real-time: O(1) extra work per plan (one extra point, one
`hypot`) — the goal is only computed on a new `/goal_pose`, not per control
tick, so this has no bearing on the MPC tail-latency work in docs 27–29.

## What this fixes vs. what it doesn't

- Fixes: the *lattice search's own* discretization gap between its accepted
  near-goal state and the true goal.
- Does **not** touch pure pursuit's independent 0.25 m stop slack — that
  still applies (same as it does for A*), so lattice+pursuit should now
  match A*+pursuit's reliability, not exceed it.

## Verification

Spot-checked headful (Gazebo GUI + RViz, `maze_astar.launch.py
planner:=lattice controller:=pursuit`) against leg 0 of the original
`lattice_pursuit_smoke` run — the leg that used to time out at 150s, robot
parked 0.28 m short at `(-1.866, -5.767)` vs goal `(-2.05, -5.55)`:

- Published `/plan`'s last waypoint is now `(-2.05, -5.55)` exactly — the
  literal goal, not a near-goal lattice state (confirmed via a one-shot TF
  subscriber, not just eyeballing RViz).
- Robot reached within 0.246 m of goal at t=16.9s (TF `map->base_footprint`
  polled every 3s). Previously: never reached, 150s timeout.

Reran the full `lattice_pursuit_smoke` (5 legs, seed 1, headless) to check
the other two failed legs too, not just leg 0:

| leg | before | after |
|---|---|---|
| 0 | TIMEOUT (150s, 0.28 m short) | REACHED, 59.9s |
| 1 | TIMEOUT (150s, 0.38 m short) | REACHED, 43.8s |
| 2 | REACHED, 40.6s (already worked) | REACHED, 45.4s |
| 3 | REACHED, 41.2s (already worked) | REACHED, 40.3s |
| 4 | TIMEOUT (150s, 0.35 m short) | REACHED, 41.8s |

**5/5 reached**, all three previously-failing legs now land in the same
~40-60s range as the two that always worked — no separate cause needed
chasing for legs 1/4 beyond the tolerance-stacking diagnosis above.
`min_clearance` stayed >= 0.3 m on every leg (`lattice_pursuit_smoke_summary.csv`),
confirming the straight docking hop isn't clipping a wall despite the "no
extra collision check" design call made above.

---

### _Interview angle_

**Q: Why doesn't a state-lattice planner just search for the exact goal
state the way A* does?**
A: A*'s move set is dense relative to the grid (every neighboring cell is
reachable), so an exact-cell goal test is cheap and always satisfiable if
the goal is free. A lattice's move set is a small, kinematically-feasible
primitive library — the reachable-state set is *sparse*: primitive
endpoints land on specific offsets, and an arbitrary goal generally isn't
one of them. So the goal test has to become "within a tolerance radius,"
with an admissible heuristic adjusted for that tolerance so A*/Dijkstra-style
search still terminates correctly (`heuristic()` in `lattice.py:178-182`
subtracts `goal_tolerance` from the raw distance estimate, keeping it a
lower bound).

**Q: If pure pursuit already has its own goal-tolerance stop, why did the
docking fix matter — doesn't 0.25 m already cover the 0.15 m gap?**
A: The two tolerances are checked against *different reference points*.
Pursuit's 0.25 m is relative to the planner's *last published waypoint*, not
the true goal. If that waypoint is already 0.15 m off, pursuit's own slack
is added on top of that offset (not "spent covering" it) — worst case the
two errors are in the same direction and sum, not cancel. Making the last
waypoint the literal goal removes one whole error term instead of counting
on the other one being generous enough by luck.

**Q: Why append a straight segment instead of re-running the primitive
search with a smaller/zero tolerance right at the end?**
A: Re-searching for an exact hit isn't guaranteed to terminate (the state
may be genuinely unreachable by any finite composition of primitives — a
diff-drive/lattice completeness issue, not a bug), and it reintroduces the
"might find zero path" failure mode from option (a). A short, unconstrained
straight hop is always achievable (given free space, which is already
locally verified) and costs nothing extra to compute.

**Q: Is this specific to diff-drive, or would it also work unchanged for
the Ackermann/Dubins lattice?**
A: Not unchanged. A car-like model can't execute an arbitrary straight hop
at an arbitrary final heading — the docking move itself would need to
respect the turning-radius constraint, i.e. a short Dubins segment from the
accepted lattice state's pose to the exact goal pose, using the same
`dubins.py` primitive machinery `lattice_ackermann.py` already builds its
edges from.
