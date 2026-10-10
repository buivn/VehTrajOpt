# 37 — Ackermann robot: EKF (localization step 1 of 2)

First of two localization pieces (EKF, then AMCL — same order diffbot
followed: docs/17 gave it the IMU, docs/18 wired the EKF, and AMCL had
already landed in docs/11 on top of a *static* `map→odom` at the time,
later upgraded once the EKF existed). For this robot both sensors already
exist (docs/36), so this step is exactly "wire the EKF" with nothing to
build first.

## 1. Theory (brief — full derivation already in docs/18)

Wheel odometry alone drifts (wheel slip, encoder quantization); IMU gyro
alone drifts too (integration of a noisy rate). An EKF fuses both: wheel
odometry's *position* is trustworthy over short spans, the gyro's *yaw
rate* is trustworthy independent of wheel slip, and a Kalman filter
combines them weighted by their respective uncertainty rather than
trusting either alone. Nothing about this derivation assumes diff-drive —
it's a property of wheel encoders and gyros in general, not of the
drivetrain converting motor speeds into robot motion.

## 2. Design: new config file, not a parametrized shared one

`robot_localization`'s `ekf_node` reads one static YAML at launch; there's
no clean way to template a single `ekf.yaml` per-robot via launch
substitutions for a scalar like `odom0: /topic` without more machinery
than the two-file alternative costs. Matches the project's own existing
pattern (`diffbot_controllers.yaml` vs `ackermannbot_controllers.yaml`) —
new file `ekf_ackermann.yaml`, not a parametrized `ekf.yaml`.

**What differs from `ekf.yaml` — exactly one line**: `odom0` points at
`/ackermann_steering_controller/odometry` instead of `/odom` (this robot's
odometry was never remapped to `/odom`, deliberately, docs/34). Everything
else — `odom0_config` (trust x/y/yaw position + vx/vyaw velocity, exclude
vy: a car can't strafe any more than a diff-drive robot can), the IMU
config (gyro yaw-rate only, same reasoning as docs/18), `world_frame:
odom` (this EKF must never touch `map→odom`, that's AMCL's job, docs 38) —
is physics that doesn't depend on drivetrain, copied unchanged.

**Required coupled change**: `ackermannbot_controllers.yaml`'s
`enable_odom_tf: true` → `false`. Diffbot's own docs/18 comment already
states why this has to be a package deal: two nodes publishing
`odom→base_footprint` simultaneously fight over the same transform. This
controller currently owns that TF directly (docs/33, since nothing else
claimed it yet); the EKF taking over is exactly the point of this step, so
the controller has to let go of it in the same change, not after.

## 3. Coding

New file: `vto_bringup/config/ekf_ackermann.yaml` (content per above).
Edit: `ackermannbot_controllers.yaml`'s `enable_odom_tf: true → false`.
New launch file: `vto_bringup/launch/maze_ackermann.launch.py` — spawns
`ackermannbot` in the maze (correct heading, docs — the corridor-facing
fix) + `ekf_node`. No planner or controller wired into it yet: no
Ackermann-compatible planner exists (A* doesn't fit this robot's
kinematics, docs/31; the Ackermann lattice's ROS node, docs 24-25, isn't
built) and there's no `/plan` for `pure_pursuit_ackermann` (docs/35) to
consume yet — adding it here would just be an idle node, not a tested
capability. A new file rather than extending `maze_astar.launch.py`:
that file's name and its `planner:=`/`controller:=` gates are specifically
A*/diff-drive-controller shaped (astar/lattice planners, pursuit/mpc/mppi
diff-drive controllers) — none of which apply here, so extending it would
mean nesting a third `robot:=` axis through conditions that don't compose
cleanly with the existing two, for a robot that shares none of the
options. `maze_astar.launch.py` and every diffbot benchmark that depends
on it stay untouched.

## Verification plan

Spawn via the new launch file, confirm: (1) `ekf_filter_node` is the one
now broadcasting `odom→base_footprint` on `/tf` (not
`ackermann_steering_controller`); (2) `/odometry/filtered` publishes at
the configured rate; (3) driving a short distance still produces sane,
monotonically-consistent pose changes (same sanity check style as docs/36).

**Results:** all confirmed. `ros2 topic info /tf --verbose` showed exactly
two publishers — `robot_state_publisher` (joint transforms) and
`ekf_filter_node` (odom→base_footprint) — with `ackermann_steering_controller`
no longer among them; `ros2 run tf2_ros tf2_echo odom base_footprint`
resolved a valid transform (previously impossible from this node's own
lookup, since that transform lived on a different topic before this
change, docs/35). `/ackermann_steering_controller/tf_odometry` no longer
has a publisher at all now that `enable_odom_tf: false`. `/odometry/filtered`
publishes at 30 Hz as configured. Drove forward 0.3 m/s briefly: filtered
position moved to (0.66, ~0), negligible lateral drift — sane straight-line
fusion.
