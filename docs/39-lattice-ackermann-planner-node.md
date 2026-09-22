# 39 — Ackermann lattice planner: the ROS node (step 3 of 3)

Closes out the 3-step Ackermann state-lattice build (docs 24–26): Dubins
edge primitive (24) → control-set enumeration + search (25) → this doc, the
ROS adapter that turns the pure-Python planner into a running node, wired
into the maze bringup end-to-end. Three changes, one story: the planner
node itself, a pose-source fix in the existing follower it feeds, and the
launch wiring that connects them.

## 1. General idea / theory

A global planner node's job in the pipeline is narrow: turn
`(map, start pose, goal pose)` into a `nav_msgs/Path`, once per goal
request. It does not run in a control loop and it does not know about
real-time deadlines — it sits strictly between localization (AMCL, which
supplies the start pose) and the local controller (pure pursuit, which
consumes the path). Steps 1–2 already built and unit-tested the actual
search — a state lattice over `(row, col, heading_bin)` where edges are
Dubins-shortest-path motion primitives (docs/24–25) instead of diff-drive's
arc/pivot set (docs/22). Step 3 has no new algorithm; it's the seam that
was left open on purpose so the search stayed framework-free and
host-testable.

Two things in this seam are genuine decisions, not boilerplate — the rest
of this doc is those two.

## 2. Method selection (the two real decisions)

### 2a. Where does `turning_radius_cells` come from?

`lattice_ackermann.py`'s `make_ackermann_primitive_library()` takes
`turning_radius_cells` as a bare argument with no physical grounding — that
was the open question left in the build plan ("pin R_min in metres vs. keep
as node param"). The answer is that it isn't a tuning knob at all: it's the
bicycle model's minimum turning radius, a hard consequence of the vehicle's
own geometry —

```
R_min = wheelbase / tan(max_steer_angle)
```

`ackermannbot.urdf.xacro` fixes both terms (`wheelbase=0.29 m`,
`max_steer_angle=0.5 rad`, `ackermannbot.urdf.xacro:32,42`), giving
**R_min ≈ 0.53 m** — the file's own comment already derives this number
independently, which is a good cross-check.

Why this matters, and why not just pick something safely large: if the
node's `turning_radius_m` were smaller than the true R_min, the search
would plan curves the steering geometry cannot physically produce — pure
pursuit would then either saturate `max_curvature` and cut the corner
short (missing the planned arc) or, worse, the arc might have hugged an
obstacle at a clearance that assumed the tighter curve was achievable. If
it were larger than R_min, the search just discards reachable, potentially
shorter primitives for no benefit — pure waste, not safety. So the only
correct default is the exact physical value, computed from the same two
xacro constants rather than hand-copied: `0.29 / math.tan(0.5)`
(`lattice_ackermann_planner_node.py`, `turning_radius_m` parameter
default). It stays overridable as a parameter since a future robot variant
would have a different wheelbase/steer limit.

I did *not* expose the other `make_ackermann_primitive_library()` knobs
(`max_heading_change_bins`, `samples_per_cell`, `redundancy_tol`,
`max_combo_depth`) as ROS parameters. Those are library-*construction*
internals — they shape how the fixed primitive set is built and pruned,
already validated by `test_lattice_ackermann.py`'s 10 passing cases at
their defaults (doc 25) — not mission-level choices an operator would ever
want to change per-launch. Exposing them would be surface area with no
real use case yet, the kind of premature configurability that just adds
noise. `turning_radius_m` and `max_reach_cells` (the search-window size,
which does trade off planning cost vs. path directness in a way worth
tuning) are the two that earn a parameter.

### 2b. Whose pose does the follower track against?

This one isn't about the new node directly — it's about
`pure_pursuit_ackermann_node.py`, the existing node the new planner feeds.
It read pose from the `ackermann_steering_controller`'s own `Odometry`
message (the **odom** frame), justified in docs/35 by a real fact: that
controller's `enable_odom_tf` publishes to its own namespaced
`<name>/tf_odometry` topic, not the shared `/tf` a `TransformListener`
reads, and — at the time — this robot had no AMCL/map at all, so odom was
the only pose available.

Both premises changed. Docs 37–38 added `ekf_node` + `amcl` for this
robot, and *both* of those publish to the shared `/tf`
(`ekf_ackermann.yaml`'s `publish_tf: true`, `amcl_ackermann.yaml`'s
`tf_broadcast: true`) — so a `map → base_footprint` lookup now resolves
fine, via that chain, not the one docs/35 ruled out. And the new planner
publishes `/plan` in the **map** frame (same convention as
`lattice_planner_node.py`). Tracking a map-frame path against an odom-frame
pose only works by coincidence of the two frames' origins; once AMCL's
`map → odom` correction is non-zero, the follower would be silently
comparing the path against the wrong pose — a drift/oscillation bug that
would look like a control-tuning problem, not a frame bug, if left as-is.

The alternative I didn't take: transform the *path* into the odom frame
instead of switching the *pose* source (what Nav2's own local controllers
actually do, precisely because AMCL corrections are discontinuous jumps
that are bad to feed straight into a real-time loop, while odom is smooth).
I'm not doing that here — it's real added complexity (a live `map → odom`
transform applied to every path point, kept in sync) for a problem this
maze doesn't have yet: AMCL updates frequently (`update_min_d: 0.2` m) in a
small environment, so corrections are small and frequent rather than
large and rare. Tracking directly in `map` frame is the simpler design for
this scale; the Nav2 pattern is worth knowing for an interview, not worth
building here.

## 3. Coding — design

**`lattice_ackermann_planner_node.py`** (new file,
`vto_planning/vto_planning/`): structurally a copy of `lattice_planner_node.py`
with two substitutions — `ackermann_primitive_table()` in place of
`build_primitive_table()`, and the physically-derived `turning_radius_m`
parameter (converted to cells via map resolution in `_on_map`, same pattern
`inflate_cells` already uses) in place of the diff-drive primitive shape
params (`step_cells`/`wide_radius_cells`/`tight_radius_cells`). Same class
shape as its sibling deliberately — same TF pose lookup, same
`densify_path` + goal-docking (`docs/30`) handling, same `/map` →
`/goal_pose` → `/plan` topic contract — because nothing about being a *ROS
adapter* changes between drivetrains; only the primitive library
underneath does. New `console_scripts` entry
(`lattice_ackermann_planner`) in `vto_planning/setup.py`.

**`pure_pursuit_ackermann_node.py`** (edited): added a `Buffer` +
`TransformListener` (same as the planner node — not factored into a shared
helper, since `vto_planning` and `vto_control` are separate packages with
no existing shared-library dependency between them, and a 6-line TF lookup
isn't worth introducing one). `_on_odom` now calls a new `_pose_in_map()`
in place of the old `_pose_from_odom(msg)`; `msg.twist.twist.linear.x`
(velocity) still comes from the `Odometry` message, since TF carries no
velocity. `global_frame`/`base_frame` parameters added, matching the
planner node's naming.

**`maze_ackermann.launch.py`** (edited): appended
`lattice_ackermann_planner` and `pure_pursuit_ackermann` as two more nodes
after the existing `ekf_node`/`map_server`/`lifecycle`/`amcl` chain — no
new `IfCondition` gate like `maze_astar.launch.py`'s `planner:=astar|lattice`,
because there's no A*-for-Ackermann alternative to gate against (`docs/31`:
A* doesn't fit this robot's kinematics at all). One planner, unconditionally
launched.

## 4. The smoke test found a real bug, not just a wiring issue

First headless run (`scripts/ackermann_lattice_smoke_test.sh`): everything up
through TF resolved cleanly, the planner received the goal and logged
`start=cell(8,8,4) -> goal=cell(290,190)`, then immediately: `"no path
found (goal blocked/unreachable)"`. Bisecting offline (no Docker needed —
`lattice.py`/`lattice_ackermann.py` have no ROS deps) against the real
maze map: the diff-drive lattice solves this exact start/goal (269
states); the Ackermann lattice fails even on a **trivial 0.5 m
straight-ahead sub-goal** with nothing in the way.

**Root cause:** `lattice.py`'s heading-index convention is "index 0 along
`+row`, rotating toward `+col`" (its own docstring). Both planner nodes'
`_cell_from_world` maps `row <- y, col <- x`. So index 0 physically means
**+y (north)**, and the index angle is measured **from the row/y-axis
toward the col/x-axis** — clockwise-from-north, not the standard
counterclockwise-from-x/east convention that a real TF yaw actually uses.
Both nodes called `heading_to_index(yaw)` directly, silently assuming
these were the same angle. They're related by a fixed twist:
`lattice_angle = pi/2 - yaw`. At the robot's actual spawn yaw (`pi/2`,
facing north into the maze's one open corridor), the raw call produced
heading index 4 — which the rotation math treats as facing **east**, the
one direction the launch file's own comment says is walled off within
0.83 m. Feeding the corrected index (0, which really is north here) into
the identical search succeeded immediately (122 states, then 128 with the
node's live map).

**Why this was invisible on the diff-drive planner:** `lattice_planner_node.py`
has the exact same call, same bug — never mattered because diff-drive has
free, zero-displacement pivot primitives (`lattice.py:99-102`+): mislabel
the start heading and the search just spends a cheap in-place turn or two
correcting it before driving off, invisible in `/plan` since that only
carries `(x, y)`, never the internal heading label. A forward-only
Ackermann vehicle has no such escape hatch — correcting a wrong heading
costs real arc space, and the spawn pocket (open only north) doesn't have
any to spare. The bug was always there; only adding a kinematic
constraint that can't cheaply undo it made it surface.

**Fix:** added `heading_index_from_yaw(yaw)` to `lattice.py` —
`heading_to_index(math.pi/2 - yaw)` — and swapped both planner nodes'
`heading_to_index(syaw)` call for it. This changes `lattice_planner_node.py`
too: its search now starts from the geometrically-correct heading instead
of a mislabeled one. It doesn't change diffbot's *feasibility* (pivoting
made it heading-agnostic either way), but it likely removes a spurious
initial pivot or two from its planned paths, which could nudge
previously-recorded lattice-vs-A* numbers (docs/14, docs/22) very slightly
— worth knowing if those are ever re-compared exactly.

Re-run after the fix: planner starts from `cell(8,8,0)`, publishes a
128-waypoint `/plan`; `pure_pursuit_ackermann` drives at a steady 100 Hz on
`/ackermann_steering_controller/reference`; odometry shows ~4 m of real
displacement within 15 s of the goal, no errors/exceptions/TF warnings in
the log.

## Interview angle

**Q: A working, verified planner suddenly failed on a new robot with no changes to the search algorithm. How do you debug that?**
A: Bisect the delta, not the whole system. The search code and map were
unchanged; only the primitive library and the vehicle's kinematics were
new — so the first move is reproducing the failure *offline*, outside ROS,
against the same inputs (map, start, goal) that failed live, since the
core planning modules here have no ROS dependency. Then narrow further:
does the *other* (working) planner solve the identical start/goal? Yes —
that isolates the fault to something the two share only partially (the
heading-index bookkeeping), not the search itself. Then test the smallest
possible case (a 0.5 m straight sub-goal) instead of the full maze — that
turned a "why can't it cross a 20x30 m maze" question into "why can't it
move half a meter forward with nothing in the way," which is a much
smaller space to explain, and pointed straight at heading-labeling rather
than the search's real cost/heuristic logic.

**Q: Why did a coordinate-convention bug survive testing on one robot but not another?**
A: Because the diff-drive robot had a capability — free in-place
pivoting — that happens to make heading-labeling errors economically free
to fix at runtime: mislabel the start heading, and the search just spends
a cheap pivot or two reorienting before it matters. That capability was
silently absorbing a real bug. The Ackermann vehicle has no equivalent
(forward-only, real turning radius costs real space), so the same bug that
was invisible for months became a hard planning failure immediately. The
general lesson: a forgiving capability elsewhere in a system can mask a
latent correctness bug for a long time; adding a stricter constraint
doesn't introduce a new bug, it just removes the thing that was
compensating for an old one.

**Q: Why is the turning radius a physical constant instead of a tuning parameter?**
A: It's the bicycle model's `R_min = L / tan(δ_max)` — a hard kinematic
limit of the vehicle, not a preference. Planning tighter than R_min
produces paths the steering geometry can't execute; planning looser than
R_min just throws away reachable, shorter primitives. The only defensible
default is computing it from the same wheelbase/steering-limit constants
the URDF already fixes, not a separately-chosen number that could drift out
of sync with the robot model.

**Q: Why does a state-lattice planner need a tolerance-radius goal test instead of matching the goal exactly, and how does that interact with a node like this?**
A: The lattice graph only contains states reachable by exact combinations
of a small, fixed primitive library from the start state; an arbitrary
continuous goal essentially never lands exactly on a graph vertex. So the
search accepts the cheapest state within `goal_tolerance_cells` of the
goal. The ROS-level consequence (`docs/30`) is that the planner node must
explicitly append the *literal* requested goal as a final waypoint after
the search result, or the residual tolerance gap (plus the controller's own
independent stop-tolerance) compounds into a real docking error.

**Q: Why not have the planner and the controller share the TF-lookup helper instead of duplicating six lines?**
A: They're two independently-launchable nodes in two separate ROS
packages (`vto_planning`, `vto_control`) with no existing shared-library
dependency. Introducing one — a new package, or a dependency edge between
the two — to deduplicate a six-line, framework-trivial TF lookup is more
structural coupling than the duplication it removes. This is the same
"three similar lines beats a premature abstraction" call as elsewhere in
this codebase; it would flip if a third node needed the same lookup, or if
the lookup grew real logic (retries, multi-frame fallback) worth getting
right once.

**Q: What's the actual difference between this planner node and the diff-drive one, code-wise?**
A: Only the motion-primitive library the search runs over — one call
(`build_primitive_table` vs. `ackermann_primitive_table`) and one set of
shape parameters (arc/pivot radii in cells vs. a physically-derived turning
radius in metres). Everything else — the `/map`→grid pipeline, TF pose
lookup, `lattice()` search call, `densify_path`, goal-docking, `/plan`
publishing — is identical, because none of it is drivetrain-specific; it's
all "how to be a ROS adapter around a pure-Python cell-space search,"
which doesn't change with the vehicle's kinematics.

**Q: Why does tracking a map-frame path against an odom-frame pose actually break, mechanically?**
A: `odom` and `map` share an origin only until AMCL's first (or next)
correction. Once `map → odom` is non-identity, a point at `(x, y)` in
`odom` frame is at a *different* physical location than the same numeric
`(x, y)` in `map` frame. Comparing a map-frame path's waypoints against an
odom-frame pose is then comparing coordinates from two different frames as
if they were the same — the follower computes a lookahead point and
cross-track error against a phantom offset equal to the accumulated
correction, which looks exactly like steady-state tracking bias or
poor tuning, not a frame bug, unless you know to check.
