# 35 — Pure Pursuit for the Ackermann robot (controller rework, step 1 of 3)

First of the three controllers (Pure Pursuit → MPC → MPPI) getting reworked
for bicycle dynamics — Pure Pursuit picked to go first, cheapest and best
motivated of the three (see "why this order" below).

## 1. Theory: what actually needs to change

The diff-drive robot has two independently-actuated wheels: (v, ω) maps
directly to (left speed, right speed), ω is a free DOF, and ω can be
nonzero at v=0 (turn in place). The Ackermann robot has exactly **2**
actuated DOF total — drive speed and steering angle δ — and every other
motion is a consequence of the bicycle model:

```
θ̇ = v·tan(δ)/L   =>   ω = v·κ,   κ = tan(δ)/L,   κ bounded by δ_max: |κ| ≤ tan(δ_max)/L
```

Two consequences that don't exist for diff-drive:
- **Curvature, not angular velocity, is what's mechanically bounded.** A car
  can produce large ω *if* v is large enough (fast + gentle turn) but never
  exceeds `κ_max` regardless of v.
- **No turn-in-place.** ω=0 whenever v=0 — there is no pivot-in-place
  recovery available for a large heading error the way there is for
  diff-drive.

Crucially: `ackermann_steering_controller` already accepts the *same*
`TwistStamped(v, ω)` reference message diff-drive's controller does, and
internally derives the required steering angle as `δ = atan(L·ω/v)`
(verified empirically in docs/33 — commanding (v, ω) reproduced the exact
unicycle-equivalent trajectory). So **the message interface doesn't need to
change at all** — what has to change is the *control law* that computes
(v, ω), so it never produces a pair that implies an unachievable δ. The fix:
compute curvature `κ` from path geometry first, **clamp it to `κ_max`
before multiplying by v**, not after. That ordering guarantees
`δ = atan(L·(v·κ)/v) = atan(L·κ) ≤ δ_max` for *any* v, including v→0 — the
division-by-v that appears in the controller's own inverse never has a
chance to demand an infeasible angle.

## 2. Method selection

**Why Pure Pursuit first, not MPC or MPPI:** MPC and MPPI each require
swapping their entire internal dynamics *model* (the ODE the optimizer/
sampler rolls out against) — for MPC-acados specifically that means
re-deriving the whole `AcadosOcp` formulation (docs 28-29), a much bigger
lift, appropriately sequenced later. Pure Pursuit needs one function's
worth of change.

**Why this is a small, well-motivated fix, not a rewrite:** Pure Pursuit's
*original* formulation (Coulter, CMU-RI-TR-92-01, 1992 — already in this
project's report references) was derived for an Ackermann vehicle in the
first place, computing steering angle directly from path curvature. This
project's existing diff-drive version (`pursuit_core.py:144`,
`curvature = 2*y_r/dist**2`) already computes that same textbook curvature
— it just adds two diff-drive-only extensions on top (turn-in-place pivot,
clamping ω instead of curvature) that have to come back out for a car. This
isn't "port pure pursuit to a new robot," it's "stop applying diff-drive-only
extensions to an already-Ackermann-shaped formula."

**Design: one new function, one new node, not a branch on the existing
ones.** `lookahead_point`/`adaptive_lookahead`/`closest_index` in
`pursuit_core.py` are pure path geometry — model-agnostic, reused unchanged.
Only the *command* law (`velocity_command`) is diff-drive-specific, so a
sibling function `velocity_command_ackermann` is added alongside it rather
than editing it in place (the diff-drive version stays exactly as verified
across every existing A*/lattice benchmark — no risk of regressing it).

A **new ROS node** (`pure_pursuit_ackermann_node.py`), not a `model:=`
branch inside the existing one, because three concrete things differ, not
just the control-law call:
- command topic: `/ackermann_steering_controller/reference`, not `/cmd_vel`
  (no remap exists for this robot, docs/34 — deliberately, until this
  rework existed)
- odometry topic: `/ackermann_steering_controller/odometry`, not `/odom`
  (same reason)
- parameters: `wheelbase`/`max_steer_angle` (→ `κ_max`), not
  `turn_in_place_angle`/`turn_in_place_gain`

This matches the project's existing convention of separate nodes per
model/planner (`astar_planner_node.py` vs `lattice_planner_node.py`) rather
than one branchy node carrying both.

## 3. Coding

**`pursuit_core.py`** — new function `velocity_command_ackermann`:
```python
def velocity_command_ackermann(target, px, py, yaw, *, max_linear, min_linear,
                               max_curvature, slow_gain):
    dx, dy = target[0] - px, target[1] - py
    x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
    y_r = math.sin(-yaw) * dx + math.cos(-yaw) * dy
    alpha = math.atan2(y_r, x_r)

    dist = max(math.hypot(x_r, y_r), 1e-6)
    curvature = clamp(2.0 * y_r / (dist * dist), -max_curvature, max_curvature)

    v = max(min_linear, max_linear / (1.0 + slow_gain * abs(curvature)))
    w = v * curvature
    return v, w, alpha
```
Same shape as `velocity_command` (reuses the same robot-frame transform and
curvature formula) with exactly the two changes theory calls for: no
turn-in-place branch, and curvature clamped *before* the `v * curvature`
multiply rather than clamping `w` after.

**`pure_pursuit_ackermann_node.py`** — same structure as
`pure_pursuit_node.py` (TF pose lookup, `/plan` subscription, adaptive
lookahead), with: `wheelbase`/`max_steer_angle` params (defaults 0.29/0.5,
matching `ackermannbot.urdf.xacro`) used to compute `max_curvature =
tan(max_steer_angle)/wheelbase` once at startup; odometry/command topics
pointed at the Ackermann controller's native names; `global_frame` default
changed to `"odom"` (this robot has no AMCL/map yet, docs/33 — defaulting
to `"map"` the way the diff-drive node does would just never find a valid
transform; revisit once localization is added for this robot).

## Verification plan

Reuse `path_publisher_node.py` unchanged (it publishes a plain
`nav_msgs/Path`, a quarter-circle arc of radius 2.0 m — model-agnostic) as
the test path, mirroring exactly how diffbot's own Pure Pursuit was first
verified in docs/05, before any planner or maze existed. Radius 2.0 m gives
curvature 0.5, comfortably under this robot's `κ_max = tan(0.5)/0.29 ≈
1.88` — an easy arc, appropriate for a first correctness check.

## Bug found during verification: TF pose lookup silently never resolved

First live run: the robot never moved at all, no errors logged (the node's
own `_robot_pose()` returns `None` on a failed lookup and just skips the
tick — silent by design, since a missing transform for one tick is normal
during startup). `ros2 topic hz` confirmed `/ackermann_steering_controller/
odometry` *was* publishing at 100 Hz, so the odometry side was fine.
`ros2 run tf2_ros tf2_echo odom base_footprint` gave the actual answer:
`Invalid frame ID "odom" ... frame does not exist`.

Root cause: **`ackermann_steering_controller`'s `enable_odom_tf` does not
broadcast onto the shared `/tf` topic.** `ros2 topic echo /tf --once`
showed only `robot_state_publisher`'s joint transforms (`base_link` →
wheel links); the odom→base_footprint transform was instead sitting on
`/ackermann_steering_controller/tf_odometry` (confirmed by echoing it
directly — type `tf2_msgs/msg/TFMessage`, correct `odom`→`base_footprint`
content). `TransformListener` only ever subscribes to `/tf` (plus
`/tf_static`), so it had nothing to listen to — a controller-namespaced TF
topic is invisible to it by construction, not a bug in `tf2_ros`. This
differs from `diff_drive_controller`, which does put its odom transform on
the shared `/tf`.

Fix: dropped the TF buffer/listener entirely and read pose straight from
the `Odometry` message's own `pose.pose` field — the node was already
subscribed to that message for `v_meas`, so this removes a dependency
rather than adding one. Also the more honest option for now: this robot
has no AMCL/map anyway, so `Odometry`'s own "odom" frame is already the
best pose estimate available; there was never a reason to prefer TF here
over the message that already carries the same data.

Re-verified after the fix: robot moved from (0,0) to (1.98, 1.75), 0.25 m
from the (2,2) target — within `goal_tolerance` — and held there (repeated
"goal reached — stopping" log lines, not oscillating).
