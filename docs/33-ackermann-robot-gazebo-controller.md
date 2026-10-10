# 33 — Ackermann robot: Gazebo + controller (step 3 of 4)

Step 3: the piece that actually makes the robot drivable — the
`gz_ros2_control` plugin block and `ackermann_steering_controller` YAML.
Verified by actually spawning and driving it, not just parsing the XML.

## Verifying param names against source, not memory

`ackermann_steering_controller` is built on `steering_controllers_library`
(`generate_parameter_library`-based). Rather than write the YAML from
recollection, I checked the installed version (`ros-jazzy-ros2-controllers
4.40.1`) and pulled the actual parameter definitions from the matching
`jazzy` branch of `ros-controls/ros2_controllers` on GitHub. Worth doing —
the older API (`front_wheels_names`/`rear_wheels_names`/`front_wheel_track`/
`front_steering`) is **deprecated** in this version in favor of
`traction_joints_names`/`steering_joints_names`/`traction_track_width`/
`steering_track_width` — guessing from older tutorials or a stale memory of
the API would have produced a YAML that silently used dead parameter names.
One non-obvious detail the source caught: both name lists are documented as
**right-joint-then-left-joint order**, the opposite of what you'd assume
from diffbot's separate `left_wheel_names`/`right_wheel_names` lists.

Also checked the command interface directly in
`steering_controllers_library.cpp`/`.hpp`: it subscribes to `~/reference`
(`geometry_msgs/msg/TwistStamped`) — so the full topic is
`/ackermann_steering_controller/reference`, a **different topic name** than
diff_drive_controller's `.../cmd_vel`, though the same message type.

## Coding

Two new files:
- `ackermannbot_controllers.yaml` (`vto_description/config/`) — same
  `controller_manager` shape as `diffbot_controllers.yaml`, but
  `ackermann_steering_controller/AckermannSteeringController` instead of
  `diff_drive_controller/DiffDriveController`, parameterized by `wheelbase`
  (0.29 as of the docs/31 length-revision, was 0.24; from the URDF), `traction_track_width`/`steering_track_width`
  (0.34, matching the URDF's `track`), `traction_wheels_radius` (0.08).
- `ackermannbot.gazebo.xacro` — same `gz_ros2_control` plugin pattern as
  diffbot, pointed at the new YAML, plus wheel friction on **all four**
  wheels at `mu=1.0`. Diffbot's front/rear are frictionless casters
  (passive support only); this robot's front wheels are real,
  load-bearing, steered wheels — cornering only works if they have grip,
  so they get the same friction as the driven rear wheels, not diffbot's
  near-zero caster friction.

Two decisions with no strong precedent to copy, so stated explicitly:
- **`enable_odom_tf: true`** — diffbot sets this `false` because `ekf_node`
  owns `odom→base_footprint` (docs/18). This robot has no `ekf_node`/AMCL
  yet (sensors are out of scope for docs 31-33), so nothing else is
  claiming that TF — the controller owns it directly, matching what
  diffbot itself would have looked like before Phase 2 introduced the EKF.
- **`position_feedback: false`** — traction (rear wheel) odometry feedback
  reads `HW_IF_VELOCITY`, matching that the rear wheels are
  velocity-commanded (docs/32), not position-commanded.

## Verification

Static (`xacro` + `check_urdf`): passes, same as steps 1-2.

Dynamic — spawned in an empty world (throwaway scratch launch file, not
committed; permanent `robot:=` bringup parametrization is step 4) and
confirmed `ackermann_steering_controller` reaches `active` state. Then
actually drove it: published `TwistStamped(v=0.3, ω=0.4)` on
`/ackermann_steering_controller/reference` for 4 s and read back odometry.

| quantity | expected (unicycle-equivalent) | measured |
|---|---|---|
| heading change | ω·t = 0.4×4 = 1.6 rad (91.7°) | 1.58 rad (90.6°) |
| chord length | 2·(v/ω)·sin(Δθ/2) = 1.07 m | 1.09 m |

Both match closely — end-to-end confirmation that the commanded
(v, ω) → the controller's internal bicycle-model inverse kinematics →
per-wheel steering angle + traction speed → Gazebo physics → odometry
pipeline is correct, not just that the robot spawns without crashing.

Still not wired into the permanent bringup (`gazebo.launch.py` is still
diffbot-only) — that's step 4.

---

### _Interview angle_

**Q: Why verify the controller's parameter names against the actual source
instead of just writing what "should" work and fixing errors as they come
up?**
A: A `generate_parameter_library`-based controller mostly fails at
*runtime* with a parameter-validation error, not at parse time — so a wrong
or deprecated parameter name wouldn't show up in `check_urdf` at all,
only when actually loading the controller. Since the deprecated and current
APIs for this exact controller happen to overlap in *shape* (both take
wheelbase/track/radius-ish parameters, just under different names), a wrong
guess here is exactly the kind of thing that fails silently or with a
confusing downstream error rather than an obvious one — worth the few
minutes to check ground truth first.

**Q: The unicycle-equivalent check (matching heading change and chord
length) — why does that work as a validity check for an Ackermann
controller specifically?**
A: Because the controller's whole job is to make an Ackermann-steered
vehicle *behave like* a commandable (v, ω) unicycle from the outside — that
abstraction is exactly why the same `TwistStamped` interface diffbot uses
also works here. If the inverse kinematics (splitting v, ω into a traction
speed and a steering angle via the wheelbase) were wrong, the robot would
still move, just not by the amount or in the direction a unicycle command
predicts — so checking against that prediction catches kinematic bugs a
"did it move at all" check would miss.
