# 36 — Ackermann robot: IMU + lidar (for a maze smoke test)

Adding the two sensors diffbot already carries (docs 17-18), scoped to what
a maze smoke test actually needs: sensors publishing sane data while
driving, not a full AMCL/EKF localization stack (that's separate, later
work — same sequencing diffbot itself followed: sensors existed for several
docs before AMCL/EKF landed on top of them).

## 1. Theory — why these two sensors, briefly

- **2D lidar**: the input AMCL's likelihood-field matching needs against the
  known map, and what a local costmap would use for obstacle avoidance —
  neither exists for this robot yet, but the sensor itself is what those
  later pieces consume.
- **IMU**: what `robot_localization`'s EKF fuses with wheel odometry to
  correct heading drift between corrections (docs 18-19) — same story,
  consumer doesn't exist yet, sensor does.

Nothing about *why* these sensors matter is car-specific — a 2D lidar
returns ranges in the horizontal plane and an IMU measures body-frame
angular velocity/acceleration regardless of how the chassis is driven.
Ackermann steering changes how the robot moves, not what it needs to sense.

## 2. Method / design: mirror diffbot's sensor blocks, don't re-derive them

No new design work here — diffbot's lidar and IMU blocks
(`diffbot.urdf.xacro:105-131`, `diffbot.gazebo.xacro:25-81`) are already
sensor specs with no diff-drive-specific assumption baked in (mount
position, noise model, Gazebo plugin config — all generic). Copying them
onto the Ackermann chassis with the same *relative* placement formulas
(inset from the front edge, roof-mounted; IMU at `base_link`'s own origin)
is the right amount of engineering here — writing a new sensor design from
scratch would be re-deriving something already verified working, for no
benefit. The only real changes are the numbers the formulas evaluate to,
since `ackermannbot`'s `base_len`/`base_hei` differ from diffbot's.

`imu_link` origin stays at `base_link`'s own origin (0,0,0) for the same
reason diffbot's does: linear_acceleration should reflect true body
acceleration, not a lever-arm/centripetal term from sitting off-center
through a turn — this matters *more* here, since Ackermann's steering
imparts more lateral acceleration through corners than diff-drive's
turn-in-place-then-arc behavior.

## 3. Coding

`ackermannbot.urdf.xacro`: two new links + fixed joints, inserted before
the ros2_control/Gazebo includes (matching diffbot's file order):
- `laser`: cylinder r=0.035 l=0.04, mounted at
  `(base_len/2 - 0.06, 0, base_hei/2 + 0.02)` — same formula as diffbot's,
  evaluates to `(0.155, 0, 0.08)` here vs diffbot's `(0.14, 0, 0.08)`.
- `imu_link`: box inertia only (no visual/collision, matching diffbot),
  mounted at `base_link`'s origin.

`ackermannbot.gazebo.xacro`: the `gpu_lidar` and `imu` sensor blocks,
copied from `diffbot.gazebo.xacro` verbatim (same topics `scan`/`imu`, same
noise model, same `gz_frame_id`s) — no parameter needed changing since
sensor physics doesn't depend on chassis shape.

## Verification plan

Spawn in the maze (correct heading from the earlier fix, docs — facing the
one open corridor at the spawn cell), confirm `/scan` and `/imu` publish at
their configured rates with plausible values (lidar ranges within
`[0.15, 12.0]` and consistent with known wall distances; IMU near-zero
angular velocity at rest, ~9.8 m/s² on one accel axis under gravity), then
drive a short distance with the Ackermann Pure Pursuit controller (docs/35)
and confirm the lidar ranges change consistently with motion — the actual
smoke test, not just "does it publish."

**Results:** all confirmed. At rest: `/scan` at 10 Hz, 360 finite beams in
[0.75, 3.28] m (consistent with the corridor's known wall distances at the
spawn cell); `/imu` at rest showed `linear_acceleration = (-0.001, -0.002,
9.805)` m/s² (gravity on the z-axis, near-zero on x/y — upright, at rest)
with non-zero covariance matching the configured noise stddev exactly
(`1.7e-2² = 2.89e-4`). Drove forward 0.93 m with a plain `TwistStamped`
command: `/scan`'s max range grew 3.28→4.19 m (more open corridor ahead,
as expected) while min stayed ~0.75 m (side walls, unchanged by forward
motion) — a coherent, motion-consistent change, not just "still
publishing." `/imu` angular velocity stayed ~2e-4 rad/s during the
straight-line drive, as it should.
