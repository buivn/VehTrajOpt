# 18 — robot_localization EKF: fusing wheel odom + IMU

Step 2 of Phase 2's state estimation work (step 1 was the IMU sensor, doc 17).
Adds `ekf_node` between `gz sim` and `amcl`, fusing `/odom` + `/imu` into
`/odometry/filtered` and taking over the `odom→base_footprint` TF. Target
wiring was planned in doc 16 §3-4 before writing any of this.

Files: [`ekf.yaml`](../ros2_ws/src/vto_bringup/config/ekf.yaml),
[`maze_astar.launch.py`](../ros2_ws/src/vto_bringup/launch/maze_astar.launch.py),
[`diffbot_controllers.yaml`](../ros2_ws/src/vto_description/config/diffbot_controllers.yaml),
[`diffbot.gazebo.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro).

---

## 1. Theory (recap, see doc 17 §1 for the full derivation)

Wheel odom drifts with *distance* (slip → compounding heading error). IMU
gyro drifts with *time*, independent of slip. A Kalman filter fuses the two
by weighting each source's contribution to the state update by its declared
uncertainty (covariance) — trust the sensor more when it says it's confident,
less when it doesn't.

## 2. Method (recap, see doc 17 §2)

`robot_localization`'s `ekf_node` — first-order linearized Kalman filter,
justified over UKF (motion model isn't nonlinear enough to need sigma points)
and over a complementary filter (no calibrated output covariance for
downstream consumers).

## 3. Design — the decisions that weren't obvious going in

**No new class.** `ekf_node` is a complete, tested executable; the actual
engineering here is configuration — deciding what to trust, how much, and in
which frame. That's still a real design step, just not a code one.

**The covariance-zero gotcha (found while verifying doc 17, fixed here):** a
Kalman gain is computed from measurement covariance. Doc 17's `/imu` had all
covariance fields at exactly zero — not the REP-145 "`-1` = unknown"
convention, just *unset*. Zero covariance reads as infinite confidence, which
either breaks the filter's math or means it blindly overwrites state with a
single noisy reading. Fix: added Gaussian `<noise>` blocks to the IMU sensor
in [diffbot.gazebo.xacro](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro)
(gyro stddev 2e-4 rad/s, accel stddev 1.7e-2 m/s² — typical low-cost MEMS
figures, a reasonable starting point, not empirically tuned). Gazebo turns
that per-axis noise model into real diagonal covariance on the outgoing
message — this was necessary, not cosmetic realism.

**`world_frame: odom`, not `map`.** This is the one parameter that, set
wrong, would have silently broken the whole two-stage design from doc 16:
`ekf_node` with `world_frame: map` tries to own `map→odom` itself, which is
AMCL's transform. Two nodes broadcasting the same frame pair fight each
other. `world_frame: odom` keeps this filter scoped to purely local fusion.

**`two_d_mode: true`.** The robot is planar and none of our sensors observe
z/roll/pitch. Leaving those states in the filter without any sensor to
correct them lets them drift unconstrained — `two_d_mode` locks them out.

**`odom0_config` — trust `x, y, yaw, vx, vyaw`, not `vy`.** A diff-drive robot
physically cannot move sideways; fusing a "measured" lateral velocity would
just be fitting integration noise, not a real signal.

**`imu0_config` — trust only `vyaw` (gyro), not `orientation` or
`linear_acceleration`.** Gazebo's simulated `orientation` is a perfect fused
value no raw IMU would give you (doc 17 §4) — fusing it would hide the exact
drift this whole exercise exists to correct. `linear_acceleration` is noisy
and not needed for the problem being solved (wheel-slip heading drift);
adding it now would widen the tuning surface for no benefit yet.

**Controllers now read `/odometry/filtered`, not `/odom`.** Handled as a
launch-time topic remap (`("/odom", "/odometry/filtered")`) in
`maze_astar.launch.py` — no changes to `pure_pursuit`/`mpc`/`mppi` node code,
since all three already just subscribe to whatever "odom" resolves to for
twist feedback (position still comes from TF `map→base_footprint`,
unaffected by any of this).

**Scope: `maze_astar.launch.py` only.** `pursuit_sim.launch.py` and
`maze.launch.py` don't run AMCL and have no `map` frame published at all —
adding `ekf_node` there wouldn't plug into anything. (Separately noticed:
`pursuit_sim.launch.py`'s `pure_pursuit` already defaults to looking up
`map→base_footprint` with no `map` frame published anywhere in that launch —
looks like a leftover from the doc-12 map-frame refactor, unrelated to this
step, not fixed here.)

## 4. Verification

Isolated container (separate `ROS_DOMAIN_ID`, same reasoning as doc 17 — kept
independent of anything else running), full `maze_astar.launch.py` headless:

- `/ekf_filter_node` appears in `ros2 node list`, alongside `/amcl`,
  `/astar_planner`, `/diff_drive_controller`, etc.
- `/odometry/filtered` publishes with **populated, finite, non-degenerate
  covariance** (position ~5e-4, yaw ~6e-4 at rest) — confirms the noise-model
  fix actually propagated through to the fused output.
- `odom→base_footprint` resolves via `tf2_echo` (identity at rest, as
  expected — robot hasn't moved from the odom-frame origin).
- Drove the robot with a raw `/cmd_vel` command: `/odometry/filtered`
  position and twist updated consistently with the applied motion (moved to
  `(0.54, 0.24)` under forward+turn, twist showed nonzero linear/angular
  matching the command) — fusion holds up under actual motion, not just at
  rest.
- No TF-conflict warnings in the launch log — confirms `diff_drive_controller`
  really did stop broadcasting `odom→base_footprint` (`enable_odom_tf: false`)
  and `ekf_node` is the sole owner.

Not yet done: a *quantitative* comparison of fused vs. raw-odom heading drift
under sustained wheel slip (e.g. a benchmark run like doc 14's, forcing slip
and comparing AMCL correction magnitude with/without the EKF in the loop).
That's the natural next check before calling this "proven better," not just
"wired correctly."

---

## Interview angle

- **Q: Why does the EKF need a noise model on the *simulated* sensor — isn't
  that just adding error on purpose?** The filter's correction step is
  mathematically driven by measurement covariance (the Kalman gain). A sensor
  with zero declared uncertainty is indistinguishable, to the filter, from a
  perfect oracle — it has no basis to weigh that measurement against its own
  prediction. Real sensors are never noise-free, so a covariance of exactly
  zero isn't "the good case," it's a missing input the filter's math depends
  on. Simulating the noise is what makes the simulated sensor behave like a
  sensor at all, mathematically.
- **Q: What would go wrong if you set `world_frame: map` here?** `ekf_node`
  would start broadcasting `map→odom` itself — the same transform AMCL
  broadcasts. Two independent broadcasters for one frame pair is undefined
  behavior in TF (`tf2` doesn't arbitrate ownership); in practice you'd see
  the transform flicker between whichever node published most recently.
- **Q: Why exclude the IMU's `orientation` field when it's right there in the
  message?** Because in this sim it's *too good* — a perfect ground-truth
  fused value, not what a real accelerometer+gyro-only IMU could produce on
  its own. Fusing it would launder away the exact heading-drift problem this
  whole step exists to solve, and the resulting filter wouldn't transfer to
  real hardware, where that field is either absent or garbage without a
  magnetometer/AHRS.
- **Q: Why remap `/odom` at the launch level instead of changing the
  controller's subscribed topic name in code?** The controllers' interface
  contract is "subscribe to odometry for twist feedback" — which *source*
  satisfies that is a deployment/wiring concern, not a controller-logic one.
  A launch-time remap keeps that decision in the launch file (where the rest
  of the pipeline's wiring already lives, doc 16) instead of hardcoding it
  into three separate node implementations.
- **Q: The EKF fuses `/odom` (which already includes AMCL's influence via TF
  lookups downstream) — is there a feedback loop risk?** No: `ekf_node`
  subscribes to the raw `/odom` *topic* from `diff_drive_controller`, which
  is upstream of and independent from AMCL entirely. AMCL only reads
  `ekf_node`'s *output* TF, one-directionally. If `ekf_node` instead consumed
  something AMCL-derived, that would create exactly the kind of circular
  correction loop good state-estimation architecture avoids.
