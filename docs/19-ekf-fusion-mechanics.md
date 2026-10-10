# 19 — How `robot_localization`'s EKF actually fuses `/odom` + `/imu`

Doc 18 covered *what* we configured and *why* (which fields to trust, which
frame to own). This doc is the layer underneath: what `ekf_node` is
mechanically doing, tick by tick, to turn those two topics into
`/odometry/filtered`. No new files — this is the internals of a node that
already exists (docs 17-18).

---

## 1. The core idea: predict, then correct

An EKF (any Kalman filter) is a recursive loop over two alternating steps,
running on an internal **state estimate** `x` (what we believe right now) and
a **covariance** `P` (how uncertain we are about it):

- **Predict**: advance `x` forward in time using a *motion model* — "given
  what I knew and how things move, where should I be now?" This step always
  *increases* uncertainty (`P` grows) — you're extrapolating, and every
  timestep without new information erodes confidence.
- **Update**: when a measurement arrives, compare it to what the model
  *predicted* that measurement should be (the "innovation" — actual minus
  expected), and pull `x` toward the measurement by an amount proportional to
  how much you trust it relative to your current uncertainty. This step
  always *decreases* uncertainty — you just learned something.

This predict/update alternation is why the filter can run its own clock
(`frequency: 30.0` in [ekf.yaml](../ros2_ws/src/vto_bringup/config/ekf.yaml))
independent of how fast `/odom` (100 Hz — `diff_drive_controller`'s
`update_rate`, [diffbot_controllers.yaml:3](../ros2_ws/src/vto_description/config/diffbot_controllers.yaml#L3))
or `/imu` (100 Hz, [diffbot.gazebo.xacro](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro))
actually publish — predicts happen on the filter's schedule, updates happen
whenever a message shows up, in between predicts.

## 2. The state vector

Internally, `robot_localization` tracks a **15-element** state per filter
instance, regardless of how many of those elements you actually feed data
into:

```
[x, y, z,  roll, pitch, yaw,  vx, vy, vz,  vroll, vpitch, vyaw,  ax, ay, az]
  position    orientation      linear vel    angular vel      linear accel
```

Every sensor's `_config` array (our `odom0_config`, `imu0_config`) is a
15-element boolean mask onto *this same vector* — "config" doesn't mean
"what this sensor measures," it means "which of these 15 slots does this
message get to correct." That's why `imu0_config` fusing only index 11
(`vyaw`) doesn't fuse an "IMU state" — it fuses *the filter's* `vyaw` state,
using the IMU as one of potentially several sources that could touch it.

`two_d_mode: true` forces `z, roll, pitch` and their velocities/accelerations
to zero *every predict cycle* — not just "don't fuse them," but actively
constrained. Without it, those states would sit at whatever they last were
(effectively zero, in the absence of anything to move them) but with
**unbounded growing covariance**, since nothing ever corrects them — harmless
numerically here, but it's dead weight in the state and a real problem the
moment any single bad measurement tries to touch one of those axes.

## 3. Predict step, concretely

At 30 Hz, before any sensor input is considered, the filter:

1. Propagates `x` forward by `dt` (~33 ms) using a constant-velocity model —
   "assume whatever velocity you last believed keeps holding for one more
   tick" (specifically it also accounts for angular-velocity-driven changes
   in heading, so the constant-*velocity* assumption is applied in the
   robot's own rotating frame, not just linear extrapolation in `x,y`).
2. Grows `P` by adding `process_noise_covariance` (left at package defaults
   here, doc 18 §3 — this is the "how much do I distrust my own prediction
   model" knob; larger values mean the filter leans on measurements faster,
   smaller values mean it trusts its own extrapolation longer between
   corrections).
3. `two_d_mode` zeroes the out-of-plane states and their covariance
   cross-terms.

This is the "where do I think I am *right now*, absent new information" half
of the loop — it runs every tick whether or not a measurement arrived.

## 4. Update step, concretely — our actual config

Whenever `/odom` or `/imu` arrives, the filter runs an update using **only**
the state dimensions that message's `_config` says to trust:

- **`/odom` update** ([ekf.yaml](../ros2_ws/src/vto_bringup/config/ekf.yaml)
  `odom0_config`): touches `x, y, yaw, vx, vyaw` — **note `vyaw` is in this
  list**, not just position/`vx`. The filter computes what it *expected*
  those 5 values to be (straight from its current state estimate — no
  transform needed, since `/odom` is already in the state's units/frame),
  subtracts the actual measurement to get the innovation, and pulls those 5
  state dimensions toward the measurement scaled by the **Kalman gain**
  `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹` — `H` here selects the 5 fused rows, `R` is
  `/odom`'s own covariance (published by `diff_drive_controller`).
- **`/imu` update**: touches only `vyaw` — the *same* `vyaw` `/odom` just
  touched. Because it's a single scalar update, the math collapses to:
  `vyaw_new = vyaw_predicted + K·(gyro_reading - vyaw_predicted)`, where `K`
  is `P_vyaw / (P_vyaw + R_vyaw)` — a literal weighted average between "what
  I predicted" and "what the gyro says." `R_vyaw` here is the tiny variance
  the Gaussian noise model added in doc 18 §3 produces (gyro stddev 2e-4
  rad/s) — much smaller than `/odom`'s reported angular-velocity variance, so
  this update pulls hard toward the gyro relative to how hard `/odom`'s
  update just pulled toward the (slip-prone) wheel-derived value.
4. After either update, `P` shrinks (`P = (I - K·H)·P`) — confidence goes up
   for the dimensions that just got corrected.

**Correction from an earlier draft of this doc:** it previously claimed
`/odom` and `/imu` "don't overlap — only `/imu` touches `vyaw`, only `/odom`
touches `vx`." That's wrong — checking `ekf.yaml` directly, **both**
`odom0_config` and `imu0_config` set `vyaw = true`. This isn't a partitioned
assignment where each sensor owns disjoint state slots; `vyaw` is a genuine
point of **overlapping fusion**, and it's the important one: every time
either message arrives, it nudges the *same* filter state, weighted by that
message's own covariance relative to the filter's current uncertainty. Since
the gyro's declared covariance is far tighter than the wheel odom's, the
filter structurally leans on the gyro for `vyaw` while still letting `/odom`
pull it back if the gyro were ever noisy or missing — this is the concrete
mechanism, not just the intent, behind "fuse wheel odom + gyro for heading."
`x, y, vx` remain `/odom`-only in the current config (doc 18's deliberate
choice not to also fuse IMU accel into `vx` yet) — that part of the earlier
description was accurate, just not the "no overlap at all" framing.

One real limitation worth being explicit about: this is **static-confidence**
fusion — `R_odom_vyaw` and `R_imu_vyaw` are both fixed numbers, not something
that adapts to what's actually happening. A live wheel-slip event doesn't
make `diff_drive_controller` report worse covariance in the moment it's
happening; the filter always weighs the two sources the same way regardless
of whether slip is currently occurring. Adaptive/innovation-based covariance
(inflating `R_odom` in real time when slip is detected) would close that gap,
at the cost of real added complexity — not something built here.

## 5. Handling out-of-order / delayed measurements

`/odom` and `/imu` are two independent ROS publishers — nothing guarantees
their messages arrive at the filter in strict timestamp order (network
jitter, scheduling, differing publish rates). `robot_localization` keeps a
short history buffer of past state+covariance snapshots; if a measurement
arrives stamped *earlier* than the most recent state already processed, it
doesn't just discard it or apply it out of order — it **rewinds** to the
buffered state at that earlier time, applies the update there, then
**replays** every predict/update since, back to now. This is what makes the
predict/update loop robust to `/odom` and `/imu` not being synchronized or
even both running at the same rate — correctness doesn't depend on
delivery order, only on each message's own timestamp.

## 6. Timeline, with our actual numbers

```
filter predict tick (30 Hz, every ~33 ms):     |    |    |    |    |    |
/odom arrivals (100 Hz, every ~10 ms):      | | | |  | | | |  | | | |  | |
/imu arrivals  (100 Hz, every ~10 ms):       ||  ||   ||  ||   ||  ||   |
```

Several `/odom`/`/imu` updates land between consecutive predict ticks — each
just nudges the current state/covariance in place; the filter doesn't need to
"wait" for a predict tick to incorporate them. `/odometry/filtered` itself is
published once per predict tick (30 Hz) — the *output* rate is decoupled from
either input rate by design.

---

## Interview angle

- **Q: What's the actual difference between "predict" and "update" in a Kalman filter, in one sentence each?** 
  Predict: propagate belief forward in time using a motion model, uncertainty grows. 
  Update: pull belief toward a new measurement by an amount proportional to relative confidence, uncertainty shrinks.

- **Q: Why does fusing `vyaw` from the IMU alone reduce to a simple weighted average, when the general EKF update is a matrix equation?** 
  Because our `imu0_config` only touches one state dimension — `H` is a 1×15 selector row, so the general `K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹` collapses to scalar arithmetic: `K = P_vyaw / (P_vyaw + R_vyaw)`. 
  The matrix form only earns its complexity when a single measurement couples multiple correlated state dimensions at once (e.g. a full pose measurement affecting x, y, *and* yaw jointly through off-diagonal covariance terms).

- **Q: Why can `/odom` and `/imu` run at the same 100 Hz and still be treated as "asynchronous" by the filter?** 
  Same nominal rate doesn't mean synchronized — they're two independent publishers with no shared clock
  edge, so their messages interleave unpredictably at the sub-tick level. The filter's design (process each message as it arrives, on its own timestamp, independent of the others) doesn't assume synchronization at all, which is what makes rate mismatches a non-issue in the first place.

- **Q: What would happen if you fused the IMU's `linear_acceleration.x` into `vx` on top of what `/odom` already fuses there?** 
  Both sources would independently pull the same state dimension toward their own measurement each time either one arrives — not a conflict exactly (each update is still a proper Bayesian correction weighted by covariance), but now `vx`'s final value depends on the *interleaving order* and relative noise characteristics of two sources instead of one, which is real added
  tuning surface — part of why doc 18 deliberately didn't do this yet.

- **Q: Why does `two_d_mode` zero the out-of-plane states every tick instead of just never fusing anything into them?** 
  Without an active correction and without measurements, those states would sit inert, correct only for exact zero *predict*-model drift/no rounding error accumulation — but their covariance would grow unbounded forever (never validated by data), setting up any downstream consumer or numerical operation that touches `P` for eventual instability. Zeroing them each tick keeps the whole 15-state vector well-defined even though only ~5 of its dimensions have anything observing them.
