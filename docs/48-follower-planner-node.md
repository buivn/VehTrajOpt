# 48 — Follower planner: ROS node + first live Gazebo run

Wiring `follower.py` (docs/47) into a real ROS 2 node and running it against
the actual robot in the 5-room maze — the first time this planner's output
has driven a physical (simulated) vehicle rather than a unit test.

## 1. General idea

Nothing new theoretically over docs/47 — this step is a pure integration
one. The interesting part is entirely in the *interface*: does the Python
module's contract line up cleanly with the ROS ecosystem it needs to sit
inside, and can it actually feed the controllers already built without
those controllers changing at all.

## 2. Method selection

**Chosen: mirror `astar_planner_node.py`/`lattice_ackermann_planner_node.py`'s
exact seam** — `/map` + `/goal_pose` + TF in, `nav_msgs/Path` on `/plan`
out — rather than invent a new topic/message shape for the denser
`(x,y,theta,v,steer)` trajectory `follow_path()` actually produces.

**Why not publish the richer state (with v, steer) on a new topic and
build a matching trajectory-tracking controller for it?** That would be
throwing away the entire point of this exercise. The whole value of
"Follower" as asked for was "feed this result into the controller we
already have" — publishing a plain `Path` costs only the `v`/`steer`
columns (discarded, same as `lattice_ackermann_planner_node.py` already
discards its own internal primitive states when it publishes `/plan`),
and in exchange `pure_pursuit_ackermann`/`mpc_controller_ackermann` need
**zero changes** to consume it. A new message type and a new controller
would be new surface area to build and debug for no benefit this task
asked for.

## 3. Coding — design

**New file `follower_planner_node.py`** (`vto_planning/vto_planning/`):
same shape as the other two planner nodes (map/TF/goal_pose subscriptions,
`planner/compute_ms` publisher for the benchmark), but with two decisions
specific to wrapping `follower.py`:

- **Unit conversion, meters → cells.** `follow_path()`'s internal
  `is_free()` call needs integer cell indices, so *everything with a
  length dimension* — `wheelbase`, `v_min`/`v_max`, `accel_min`/`accel_max`
  — is divided by the map's resolution before being handed to
  `plan_follower()`. `max_steer_angle` and `steer_vel_min`/`max` are
  angular, so they pass through unconverted. This is the exact same move
  `lattice_ackermann_planner_node.py:151` already makes for
  `turning_radius_cells`; the difference here is there are *several* such
  quantities instead of one, since the bicycle-model layer has more
  physical parameters than a lattice's precomputed primitive table does.
- **`theta` needs no conversion at all.** Cell axes (row‖y, col‖x) share
  the same orientation as world axes, just a different scale and origin
  offset — no rotation between them. `atan2(dy, dx)` therefore returns the
  identical angle whether `dx, dy` are expressed in meters or in cells
  (the scale factor cancels). This is a materially different, and simpler,
  situation than `lattice.py`'s heading-*bin* convention, which has a real
  π/2 twist baked into how it discretizes headings (docs/39) — that fix
  doesn't apply here because nothing here is binned.
- **Publishing on partial failure.** `astar_planner_node`/
  `lattice_ackermann_planner_node` publish nothing when planning fails.
  This node instead publishes whatever *prefix* of `follow_path()`'s
  states was collision-free, even when `reached=False`. `follow_path()`
  already computed and returned that prefix; discarding it on a ROS
  failure path would throw away exactly the information that made the
  A*+MPC stall (docs/44) worth keeping as a report finding rather than a
  bug to silently swallow. The robot then visibly drives up to, and stops
  at, the real failure point.
- **Real heading in `/plan`'s poses.** `astar`/`lattice_ackermann` publish
  identity orientation on every pose (verified: neither
  `pursuit_core.py`'s `lookahead_point`/`sample_reference` nor the MPC's
  reference-sampling reads pose orientation — only positions). This
  planner's states carry a genuinely simulated heading at every point, so
  its `/plan` poses set real orientation from `theta` — a free
  visualization improvement in RViz, not a functional one.

**Launch wiring:** `maze_ackermann.launch.py` gets a third `planner:=`
value, `follower`, gated the same `IfCondition(EqualsSubstitution(...))`
way `lattice`/`astar` already are — no other launch file logic changes.

## Verification — first live Gazebo run

Built (`colcon build --packages-select vto_planning vto_bringup`, clean),
regression-checked (`test_follower.py`, all 9 pass unchanged in-container),
then launched `maze_ackermann.launch.py world_sdf:=maze_rooms.sdf
world_map:=maze_rooms.yaml planner:=follower controller:=pursuit
headless:=false` and sent one `/goal_pose` at a room center, (7.44, -5.80),
from spawn (-9.1, -14.1):

- `follower_planner`: `planning: start map(-9.10,-14.10,yaw=1.57) -> goal
  map(7.44,-5.80)`, then `published /plan: 1180 states (map frame),
  reached=True` — **168 ms** total (A* search + the full PID/bicycle-model
  simulation over the whole route).
- `pure_pursuit_ackermann`: received the 1181-point path, drove it, then
  logged `goal reached — stopping` ~77 s later.
- Ground truth check: `map->base_footprint` (AMCL+EKF-corrected TF) shows
  the robot stationary at **(7.234, -5.851)** — 0.24 m from the commanded
  goal, within `pure_pursuit_ackermann`'s own 0.25 m tolerance.

One verification wrinkle worth recording: reading the raw
`/ackermann_steering_controller/odometry` topic directly afterward showed
a wildly different, seemingly out-of-map-bounds number. That topic is
**not** map-frame — it's the controller's own wheel-odometry integration,
anchored to wherever it happened to zero at boot, unrelated to the map's
coordinate origin (exactly the odom-vs-map distinction docs/37-39 already
exist to correct for). The TF lookup above is the one that matters; the
raw odometry number was never a bug, just the wrong topic to trust for
ground truth — a useful reminder to check *which* frame a number is
actually in before treating it as a red flag.

## Interview angle

**Q: Why does the node convert several physical constants to cell units,
where `lattice_ackermann_planner_node.py` only had to convert one
(`turning_radius_m`)?**
A: It's the same underlying requirement (whatever a downstream collision
check indexes a grid with must be in cell units), it just surfaces more
often here because `follower.py`'s bicycle model carries more independent
physical parameters — wheelbase, velocity bounds, acceleration bounds —
than a lattice's precomputed primitive table does. A lattice already
bakes its geometry into the table once, in cell units, at table-build
time; this planner instead simulates the model live for every goal, so
every quantity the model needs has to be in the same units the grid it's
being checked against uses, every time.

**Q: Why does `theta` skip the unit conversion that positions and
velocities need?**
A: Because it isn't a length — cell row/col axes are just a scaled,
translated copy of the map's x/y axes, with no rotation between them. A
scale factor multiplies both components of any (dx, dy) vector equally,
so `atan2(dy, dx)` is invariant to it. The π/2 fix `lattice.py` needed for
its heading bins is solving a completely different problem: that
discretization scheme assumed heading index 0 pointed along +row, but
real yaw = 0 points along +x — an actual axis mismatch, not a units one.
No such assumption exists here.

**Q: Why publish a partial trajectory on failure instead of nothing, like
the other two planners do?**
A: The other two planners fail as a single atomic yes/no from A*/the
lattice search — there's no partial result to speak of; either a full path
exists or it doesn't. This planner fails differently: it can plan
successfully (A* found a route) and then discover mid-simulation that the
route isn't actually drivable at some specific point. Silently discarding
that partial trajectory would turn an informative, specific failure
(*here* is where it broke, and why) into an uninformative blank one — the
opposite of the reporting value this planner is meant to add over a
success/fail-only search.

**Q: The raw odometry topic showed the robot 11 meters from where TF said
it was. How do you know the run actually succeeded and this isn't a bug
you're glossing over?**
A: Because those two topics describe different frames, and I checked
which one is ground truth before drawing a conclusion, rather than react
to whichever number I saw first. `/ackermann_steering_controller/odometry`
is raw wheel-odometry integration anchored at boot in an arbitrary local
frame; the map-frame TF (AMCL+EKF-corrected) is the one that's actually
tied to the map's coordinate system the goal was specified in, and it
matched the goal to within 0.24 m, consistent with the controller's own
"goal reached" log. If the TF pose itself had disagreed with the goal,
that would have been a real finding worth chasing; a raw odometry number
being far from a map-frame goal is expected, not evidence of anything
wrong.
