# 17 — Adding an IMU (Phase 2, step 1)

Adds a simulated IMU to the diffbot and gets it publishing on `/imu`. This is
step 1 of Phase 2's state estimation work — no fusion yet, just the raw sensor.
Step 2 (next) is the `robot_localization` EKF that consumes this + wheel odom.

Files: [`diffbot.urdf.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.urdf.xacro),
[`diffbot.gazebo.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro),
[`bridge.yaml`](../ros2_ws/src/vto_simulation/config/bridge.yaml),
[`empty.sdf`](../ros2_ws/src/vto_simulation/worlds/empty.sdf) /
[`generate_maze.py`](../ros2_ws/src/vto_simulation/scripts/generate_maze.py).

---

## 1. Why an IMU at all

Wheel-encoder odometry (`diff_drive_controller`, already running) integrates
wheel velocity into pose. Every error source — slip, tire deformation, encoder
quantization, wheel-radius/separation miscalibration — accumulates *without
bound*, and heading error is the expensive one: a small yaw error rotates all
later translation in the wrong direction, so position error grows with
distance traveled, not just time.

An IMU's gyroscope measures angular velocity directly in the body frame,
independent of whether the wheels are slipping. It has its own drift (bias
integrated over *time*, not distance), but that's a different, complementary
failure mode — which is exactly what makes it worth fusing with wheel odom
rather than replacing it.

Pipeline placement: **state estimation**, upstream of both AMCL (which
consumes `odom→base_footprint` as its motion-model input, doc 11) and any
controller reading odometry. A tighter local odom estimate means a tighter
AMCL predict step and cleaner velocity feedback for MPC/MPPI.

---

## 2. Method selection (recap — decided at repo level, justified here)

**`robot_localization`'s `ekf_node`** (next step) fuses `/odom` + `/imu`.

- **EKF, not UKF** (`robot_localization` offers both): the diff-drive motion
  model is only mildly nonlinear over one filter timestep (small-angle trig).
  First-order linearization is accurate enough; UKF's sigma-point propagation
  buys robustness against nonlinearity we don't have, for more compute — and
  CLAUDE.md already reserves GPU for MPPI/RL, not this.
- **Not a complementary filter**: fine for single-sensor attitude fusion, but
  doesn't produce a calibrated joint covariance over the full pose/twist
  state — which AMCL's particle weighting and any future added sensor want.
- **Not "just calibrate the wheels harder"**: no wheel-only signal corrects
  slip-induced heading drift. That's the gap the IMU fills.
- **Not fusing yet**: can't tune a filter against a sensor whose real output
  you haven't inspected. Hence this step being sensor-only.

---

## 3. Design — this step

Mirrors the existing lidar pattern (doc 07's planner needs the lidar; here the
IMU is added the same structural way):

- **`imu_link`** ([diffbot.urdf.xacro](../ros2_ws/src/vto_description/urdf/mobile/diffbot.urdf.xacro)):
  fixed joint to `base_link`, placed at `base_link`'s own origin — *not*
  offset like the lidar. An IMU mounted away from the center of mass picks up
  lever-arm/centripetal acceleration during turns that isn't true body
  acceleration, which would corrupt fusion later.
- **Gazebo `<sensor type="imu">`** ([diffbot.gazebo.xacro](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro)):
  `<topic>imu</topic>`, 100 Hz, `gz_frame_id=imu_link`.
- **Bridge** ([bridge.yaml](../ros2_ws/src/vto_simulation/config/bridge.yaml)):
  `GZ_TO_ROS`, `gz.msgs.IMU` → `sensor_msgs/msg/Imu` on `/imu`, same pattern
  as the existing `/scan` entry.
- **World plugin — the one real gotcha**: the lidar is a *rendering* sensor,
  handled by `gz-sim-sensors-system` (already loaded). The IMU is **not**
  rendering-based and has its own system plugin, `gz-sim-imu-system` /
  `gz::sim::systems::Imu`. Without it in the world SDF, the sensor element
  parses fine and the topic even appears in `ros2 topic list` (bridge doesn't
  care), but nothing is ever published on it — a silent failure, not an
  error. Added to `empty.sdf` directly and to `generate_maze.py`'s template
  (then regenerated `maze.sdf` — it's a build artifact per doc 06, not
  hand-edited).

No new node/class this step — pure config, consistent with "get real sensor
data flowing before writing anything that consumes it."

---

## 4. Verification

Spawned the robot in an isolated container (separate `ROS_DOMAIN_ID`, to
avoid colliding with an unrelated benchmark already running in the shared dev
container) and echoed `/imu` at rest:

```
orientation:         ≈ identity quaternion   (robot not rotated)
angular_velocity:    ≈ 0                     (robot stationary)
linear_acceleration: x≈0, y≈0, z≈9.8         (gravity, proper-acceleration convention)
```

All physically correct. One thing carried into step 2: all three covariance
blocks come back **zero**, not the REP-145 "`-1` = unknown, don't use"
convention — Gazebo's IMU sensor isn't injecting a noise model by default. The
EKF config can't trust the message's own covariance fields; it'll need
explicit values (either a Gaussian noise block added to the SDF sensor, or
override covariances directly in the `ekf_node` config).

---

## Interview angle

- **Q: Why fuse an IMU with wheel odometry instead of using either alone?**
  They have independent, complementary failure modes: wheel odom drifts with
  *distance* (slip compounds into heading error), IMU drifts with *time*
  (gyro bias integration). Fusing lets each cover the other's weak spot over
  the timescale where it's still accurate.
- **Q: What does `sensor_msgs/msg/Imu` actually contain?** Three 3-axis
  measurements — `orientation` (quaternion), `angular_velocity` (gyro,
  rad/s), `linear_acceleration` (accelerometer, m/s², *includes* gravity) —
  each with its own 3×3 covariance.
- **Q: Why is `orientation_covariance[0] = -1` a convention worth knowing?**
  REP-145: a raw IMU without a magnetometer/AHRS can't produce absolute
  orientation on its own; `-1` tells consumers "don't use this field." Sim
  IMUs often give you a suspiciously perfect fused orientation for free —
  worth deciding deliberately whether to trust it or discard it to match
  real-hardware behavior.
- **Q: Where should the IMU physically mount, and why does it matter?** As
  close to the center of mass as possible. Off-center placement adds a
  lever-arm term — centripetal/tangential acceleration from rotation — into
  the accelerometer reading that isn't the body's true linear acceleration,
  which would bias the fusion if not explicitly modeled.
- **Q: EKF vs UKF for this fusion?** EKF linearizes the motion model with a
  first-order Taylor expansion around the current estimate — cheap, and
  sufficient when the model is only mildly nonlinear per timestep (true for
  diff-drive kinematics). UKF propagates sigma points through the *exact*
  nonlinear model — more accurate under strong nonlinearity/discontinuities,
  at higher compute cost. Pick UKF when linearization error would actually
  bite; here it wouldn't.
- **Q: Is `/odom` here ground truth from Gazebo?** No — `gz_ros2_control`
  exposes simulated wheel joint states through the same `ros2_control`
  hardware-interface abstraction real encoders use; `diff_drive_controller`
  runs the identical wheel-odometry integration on top, in sim or on real
  hardware. `/odom` was never ground truth; it's already a drift-prone
  estimate, which is the whole reason step 2 exists.
