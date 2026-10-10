# 34 — Ackermann robot: bringup wiring (step 4 of 4)

Final step: make `robot:=ackermannbot` a real launch-time switch instead of
a hardcoded diffbot path, so both robots can be spawned from the same
launch files. This closes out the Ackermann build started in docs 31-33.

## Coding

Two files touched, both existing (no new files this step):

- **`description.launch.py`** — the xacro path was `diffbot.urdf.xacro`
  literally; now `[LaunchConfiguration("robot"), ".urdf.xacro"]` inside a
  `PathJoinSubstitution` (string concatenation within one path segment,
  standard launch-substitution idiom), with `robot` a new launch arg
  (`default_value="diffbot"`, so nothing that omits it changes behavior).
- **`gazebo.launch.py`** — three changes:
  1. `robot` threaded through to `description.launch.py`'s include.
  2. The spawn entity name (`ros_gz_sim create -name ...`) was the literal
     string `"diffbot"`; now `LaunchConfiguration("robot")`.
  3. The controller spawner: previously one unconditional
     `diff_drive_controller` spawner node. Now two spawner nodes gated by
     `IfCondition(EqualsSubstitution(robot, "diffbot"))` /
     `IfCondition(EqualsSubstitution(robot, "ackermannbot"))` — the exact
     same gate pattern `maze_astar.launch.py` already uses for
     `planner:=astar|lattice` and `controller:=pursuit|mpc|mppi`, not a new
     one invented for this file.

**Deliberately no `/cmd_vel` remap for the Ackermann spawner.** Diffbot's
spawner remaps `diff_drive_controller`'s namespaced topics to
`/cmd_vel`/`/odom` so existing Pure Pursuit/MPC/MPPI nodes connect without
per-consumer wiring. Doing the same for `ackermann_steering_controller`
would be misleading right now: a plain `(v, ω)` Twist still can't drive
this robot correctly (no turn-in-place, docs/31), and those controllers
haven't had the bicycle-dynamics rework yet. Remapping to the same topic
name as diffbot would imply a compatibility that doesn't exist. Leaving it
on the native `/ackermann_steering_controller/reference` name is the
honest state of things — a future step deliberately opts in by wiring a
new remap once the controller-side rework lands, rather than silently
inheriting one that predates it.

## Verification

Ran both robots through the **actual committed launch files** (not a
throwaway script, unlike step 3's spot-check) and confirmed both reach
`active` controller state:

- `ros2 launch vto_simulation gazebo.launch.py headless:=true` (default,
  no `robot:=` arg) — **regression check**: `diff_drive_controller` active,
  `/cmd_vel` and `/odom` still present as before this change.
- `ros2 launch vto_simulation gazebo.launch.py headless:=true
  robot:=ackermannbot` — `ackermann_steering_controller` active; topic list
  shows only `/ackermann_steering_controller/{reference,odometry,
  tf_odometry}` (no stray `/cmd_vel`, confirming the "no remap" design
  choice took effect); `/joint_states` lists all 6 expected joint names.

This closes docs 31-34: the Ackermann robot spawns, drives (step 3's
kinematic check), and is selectable from the same bringup entry point as
diffbot via one launch argument.

## What's still open (not this build)

- No sensors (lidar/IMU) or localization (AMCL/EKF) yet — deferred until
  this robot needs them, same as diffbot's own sensor additions came well
  after its initial build.
- `maze_astar.launch.py`/`bench.launch.py` (the planner+controller+AMCL
  stack) are untouched — out of scope until sensors exist and, more
  fundamentally, until PP/MPC/MPPI get the bicycle-dynamics rework (memory:
  "Expand controllers to different robot models") and A* is swapped for
  the already-partially-built Ackermann lattice (`lattice_ackermann.py`,
  docs 24-25) for planning, since A*'s pivot-turn paths don't fit this
  robot's kinematics.

---

### _Interview angle_

**Q: Why gate the controller spawner with two conditional nodes instead of
one node whose `arguments` list is built dynamically from `robot`?**
A: `launch` substitutions resolve at launch time, not at Python
authoring time — you can't branch Python control flow (`if robot ==
"diffbot"`) on a `LaunchConfiguration`, since its value isn't known until
the launch file is actually processed. The two-static-nodes-plus-condition
pattern is how `launch` represents "pick one of these at launch time," and
it's already the established idiom in this same codebase
(`maze_astar.launch.py`), so reusing it here means the whole `vto_bringup`
launch layer stays consistent rather than mixing two different ways to
express the same kind of choice.

**Q: Isn't leaving the Ackermann controller un-remapped just a missing
feature — why frame it as a deliberate decision?**
A: Because the *available* fix (adding the remap) is easy and would make
the launch layer *look* symmetric with diffbot's, but symmetry in topic
names isn't the same as symmetry in what the robot can actually do with a
message on that topic. Adding the remap now would make it trivial to
accidentally point an unmodified Pure Pursuit/MPC/MPPI node at this robot
and get silently-wrong behavior (commanded turn-in-place saturating at the
steering limit) instead of an honest "topic not found." The absence of the
remap is doing real work: it's the thing that would force whoever wires
this next to go read why, rather than assume it's ready.
