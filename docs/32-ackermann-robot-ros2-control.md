# 32 — Ackermann robot: ros2_control interfaces (step 2 of 4)

Step 2: declare which joints are actuated and how (`ros2_control` hardware
interface block), following the same shape as `diffbot.ros2_control.xacro`.
Still no Gazebo plugin/controller YAML — the interfaces declared here are
inert until step 3 loads a controller that claims them.

## Design decision: rear-wheel drive, front-wheel steering

Diffbot's two wheels are both driven and both steer (implicitly, via
differential speed) — one command interface type (`velocity`) on both
joints. Ackermann has to split that: **which joints get `velocity` (drive)
and which get `position` (steer) is a real decision, not mechanically
forced by the body built in step 1** (that URDF just has 4 wheels + 2
steering joints; it doesn't say which wheels are driven).

Picked **rear-wheel drive**: the two `rear_*_wheel_joint`s get `velocity`
command interfaces (traction), the two `front_*_steer_joint`s get `position`
command interfaces (steering), and the front wheels' own spin joints
(`front_*_wheel_joint`) are state-only — they roll passively, driven by
ground contact, not commanded.

Why, not just "conventional default":
- The advisor's own bicycle-model code (`MPSnakeSimulator.cpp:156-167`)
  branches on `m_frontWheelDriving`; the non-front-wheel-driving branch is
  the simpler one (`ds[STATE_X] = v*cos(theta)`, no `cos(steer)`/`sin(steer)`
  coupling into the velocity terms) — i.e. **the same bicycle-model equation
  this project already uses elsewhere assumes rear-wheel driving unless
  told otherwise.** Matching that keeps this robot's kinematics consistent
  with `MPStandardSimulator`'s model instead of silently introducing a
  different one.
- `ackermann_steering_controller` (`steering_controllers_library`) is
  conventionally configured this way too — traction on the rear axle,
  steering on the front — so this doesn't fight the controller's own
  assumptions going into step 3.

## Coding

New file: `ackermannbot.ros2_control.xacro`, same directory, same shape as
`diffbot.ros2_control.xacro` (one `<ros2_control name="..." type="system">`
block, `gz_ros2_control/GazeboSimSystem` hardware plugin, one `<joint>`
block per actuated/sensed joint).

| joint | command interface | state interfaces | why |
|---|---|---|---|
| `front_left_steer_joint`, `front_right_steer_joint` | `position`, limited to `±max_steer_angle` (same 0.5 rad from the URDF joint limit — not a new number) | position, velocity | steering actuation |
| `rear_left_wheel_joint`, `rear_right_wheel_joint` | `velocity`, `±20.0` (same generous ceiling diffbot uses — the real behavioral limit lives in the controller YAML, step 3, same split diffbot already has between its `±20.0` hardware ceiling and `diff_drive_controller.yaml`'s `linear.x.max_velocity: 1.0`) | position, velocity | traction |
| `front_left_wheel_joint`, `front_right_wheel_joint` | *(none)* | position, velocity | passive rolling — no actuation, but velocity/position still exposed as state so odometry or diagnostics could read them later |

`ackermannbot.urdf.xacro` gets one new line — `xacro:include` of this file,
in the same slot diffbot's does (after the body, before Gazebo wiring,
which is still step 3/4).

Verified: `xacro` render + `check_urdf` both pass on the combined file
after wiring the include in.
