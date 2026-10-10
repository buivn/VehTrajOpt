# 47 — Follower planner: A* guidance + PID/dynamic-bicycle-model trajectory generation

Porting the advisor's "Follower" idea from `src/GP` into a Python module
for the Ackermann robot: layer 1 (A*) gives a coarse, collision-aware
route; layer 2 (PID + the vehicle's actual dynamics) turns it into a
dense trajectory that is kinematically *and* dynamically feasible by
construction, collision-checked at every simulated step.

## 1. General idea / theory

A*'s output is a sequence of grid waypoints — geometrically valid, but it
knows nothing about the vehicle: no heading, no turning radius, no limits
on how fast speed or steering angle can change. Layer 2's job is to turn
that into something a real (or simulated) car can actually drive, by
**simulating the vehicle's own equations of motion** toward each waypoint
in turn, rather than post-hoc curvature-clamping a geometric path (Pure
Pursuit) or solving a constrained optimization over it (MPC). Because the
output trajectory comes from literally integrating the model, it is
feasible by construction — the only way it "goes wrong" is a genuine
collision, not a validity check discovered after the fact.

Where this sits relative to what already exists: `lattice_ackermann`
(docs/39/41-42) builds feasibility into the *search* (a precomputed
primitive library, one lattice edge per motion); Follower instead builds
it into a *forward simulation* driven by feedback control, decoupled from
any discrete edge library. Same goal — a drivable path — different
mechanism entirely.

## 2. Method selection

- **Read the actual C++ first, not just the description.** The advisor's
  `FollowPlanner` (`src/GP/FollowPlanner.{hpp,cpp}`, `Follow.{hpp,cpp}`)
  is a **kinodynamic RRT**: a randomized tree search biased by a "follow
  corridor" (a sequence of quad-shaped regions around a graph shortest
  path, `Follow::ComputeBoundaries`), with fallback RRT/EST/GUST
  exploration modes when following stalls (`FollowPlanner::Run`'s state
  machine). That is a materially bigger thing than "A* guidance, then
  PID+model generates a feasible trajectory" — the PID+model piece is
  real and present (`MPSnakeSimulator::SteerToPosition`/`MotionEqs`,
  called once per simulated step inside the RRT's `ExtendFrom`), but it's
  the *local steering primitive* the RRT calls repeatedly, not the whole
  algorithm.
- **Chosen: port the PID+model steering law as a deterministic sequential
  waypoint-chaser, not the RRT wrapper around it.** This matches what was
  actually asked for, reuses the same faithful robot-model/PID pieces,
  and is a much smaller, well-scoped port. Its own collision check
  converts "would have needed the RRT's robustness to route around this"
  into an honest, visible failure (stop, report `reached=False`) rather
  than silently producing something unsafe.
- **Rejected (for now): the full RRT-with-follow-corridor port.** Real
  research-grade robustness, but substantially more code (tree
  management, corridor sampling, group-weighted vertex selection, three
  fallback exploration modes) than this task calls for. Parked, not
  discarded — see docs/25's "designed but not built" precedent for how
  this project already treats scoped-out future work.

Two deliberate departures from the ported C++, both to fit *this* robot
rather than the advisor's generic research one (same discipline as
docs/39's turning-radius derivation and docs/31's steering-angle pick):
- **Rear-wheel-driving bicycle equations** (`dtheta = v*tan(steer)/L`),
  not `MPSnakeSimulator`'s front-wheel-driving default
  (`dtheta = v*sin(steer)/L`). `ackermannbot.urdf.xacro` drives its rear
  wheels and steers the front ones (docs/31) — the same convention
  `pure_pursuit_ackermann`'s `max_curvature = tan(max_steer_angle)/L`
  already assumes.
- **This project's own physical limits**, not `GPConstants.hpp`'s
  `ROBOT_*` defaults (1.75 m body, ±88° steering, ±3 rad/s steering
  velocity) — those are explicitly a different, larger scenario robot's
  unconstrained-testing numbers (docs/31 already made this exact call for
  the steering angle). `accel`/`steer_vel` limits (±0.3 m/s², ±1.0 rad/s)
  are picked fresh at this robot's scale, flagged as picked-not-measured
  the same way docs/39 flagged its own defaults.

## 3. Coding — design

**New file `follower.py`** (`vto_planning/vto_planning/`, framework-free,
same convention as `dubins.py`/`reeds_shepp.py`/`lattice.py`):
- `PIDController` — direct, unmodified port of `Utils/PIDController.hpp`.
- `BicycleModel` — port of `MotionEqs` + `SimulateOneStep` (explicit
  Euler, matching the original's own integration scheme exactly). State
  `(x, y, theta, v, steer)`; control `(accel, steer_vel)`. Velocity and
  steer angle are **states**, not instantaneous commands — the single
  most important structural fact carried over from the port (see
  interview angle below). Clamps the resulting `v`/`steer` to physical
  limits after each step — a deliberate departure from the original,
  which instead *rejects* an out-of-bound state as invalid (fine for an
  RRT that can try a different random branch; too brittle for this
  module's single deterministic pass, where a marginal one-step overshoot
  would otherwise kill the whole trajectory for no benefit).
- `steer_to_waypoint` — port of `SteerToPosition`: two **independent**
  PID loops, one driving heading error to zero (→ steer_vel), one driving
  velocity toward a cruise setpoint (→ accel). The heading-error sign
  convention was derived from `Algebra2D::VecFromToAngleCCW`'s actual
  formula and checked against a worked example (facing +x, target at +y
  → heading_error=+90°, correctly commanding positive/left steer_vel) —
  not assumed from the C++ alone, since getting a PID's error sign
  backwards produces something that looks plausible right up until it
  doesn't.
- `follow_path` — the sequential chaser: advances through a waypoint list,
  simulating step-by-step, collision-checking each step against the same
  inflated-grid + point-check convention `astar.py`/`lattice.py` already
  use (not a new collision-checking scheme).
- `plan_follower` — top-level: A* (caller-supplied, so it can be the
  project's existing `astar()` with whatever connectivity/clearance
  settings) for layer 1, `follow_path` for layer 2.

## Verification

9 tests (`test_follower.py`), including two load-bearing ones:
- `test_bicycle_model_turning_radius`: a constant nonzero steering angle
  must trace a circle of radius `L/tan(steer)` — the textbook bicycle-
  model invariant, checked independently of the derivation, not just
  "does it run."
- `test_no_stuck_at_v0_with_large_heading_error`: reproduces the *exact*
  scenario that trapped the curvature-constrained MPC (docs/44) — a
  ~180° heading mismatch at a standing start. Result: `accel=0.300`
  (saturated) and `steer_vel=1.000` (saturated) — both PIDs push hard
  immediately, empirically confirming the claim made before writing any
  code, not just arguing it.

Plus: sign-convention check, straight-line and obstacle-blocked scenarios
(confirms it actually stops at a wall, not just claims to), a steer-limit
audit over an aggressive zigzag, and one A*+follower end-to-end run
through a gap in a wall. All 9 pass; the rest of the pure-Python suite
(`test_lattice.py`, `test_lattice_ackermann.py`, `test_dubins.py`,
`test_reeds_shepp.py`) still passes unchanged — nothing shared was
touched.

**Not done yet:** wiring this into a ROS node (a `follower_planner_node.py`
publishing `/plan`, or publishing the denser `(x,y,theta,v,steer)`
trajectory directly for a future trajectory-tracking controller instead
of a path-tracking one) and running it against the actual maze map in
Gazebo. PID gains are the advisor's own defaults, carried over as a
starting point, not yet tuned/verified at this robot's scale.

## Interview angle

**Q: Why doesn't this approach hit the same "stuck at v=0" failure the MPC did?**
A: The MPC's failure came from a hard constraint (`|w| ≤ max_curvature·v`)
that *couples* heading-rate to speed — at `v=0`, `w` is forced to exactly
0, so there's no way to even begin correcting a bad heading without first
moving, and moving (in the wrong direction) only made tracking cost
worse, trapping a single-shot optimizer at the origin. This model treats
steering angle as its own **state**, independent of velocity — physically,
"you can turn the steering wheel while parked." The two PID loops here
are fully decoupled: the velocity loop doesn't look at heading error at
all, so it always pushes toward cruise speed; the steering loop doesn't
look at velocity, so it can "pre-aim" the wheel toward the correct
direction while the car is still at `v≈0`. Neither loop has a mechanism
to rationally settle on zero output indefinitely.

**Q: Does that mean this approach can't fail the way A*+MPC did?**
A: It can't fail via that *specific* local-minimum mechanism, but it's
not immune to failure in general — it can still collide. If a
reorientation genuinely needs more physical room than the map provides
(the same "forward-only vehicle needs room to turn" constraint behind
docs/40's heading-9 trap), this reactive controller has no lookahead to
see that coming; it just drives until the collision check stops it. Same
underlying vehicle limitation, different failure signature — a visible,
honest collision-and-stop instead of a silent freeze.

**Q: Why port the PID+model steering law but not the RRT search around it?**
A: They solve different problems. The RRT with a follow-corridor exists
to be *robust* — if a straight chase toward the guidance path would fail,
the tree search can back off and explore an alternative route instead.
The steering law itself (PID driving a dynamic bicycle model) is a
*local* controller, useful on its own for exactly the deterministic case
this task asked for: turn one A* path into one feasible trajectory. Where
the deterministic version fails (collision), that's precisely the
information a future step could use to decide whether the extra
complexity of the full RRT is actually worth adding — measuring first,
same discipline as this whole session's approach to the lattice-vs-A*
speed trade-off.

**Q: Why simulate the model explicitly instead of just computing a
smooth path (e.g., a spline) through the A* waypoints?**
A: A spline can be made geometrically smooth without ever being checked
against the vehicle's actual actuator limits (how fast the steering wheel
can turn, how hard the car can accelerate) — smoothness in shape doesn't
imply feasibility in time. Simulating the real model with rate-limited
controls forward means every single state in the output trajectory was
reached by a control the vehicle could actually issue at that rate; the
trajectory's feasibility isn't a property to verify afterward, it's a
consequence of how it was built.
