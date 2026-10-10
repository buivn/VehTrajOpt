# 02 — ros2_control: From `/cmd_vel` to Spinning Wheels

Doc 01 built the robot's *body*. Now we build its *muscles + reflexes*: the layer
that turns a high-level command ("drive forward at 0.5 m/s while turning left")
into actual wheel velocities, and reads the encoders back to produce odometry.

Two files this time:
[`diffbot.ros2_control.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.ros2_control.xacro)
(the hardware *interface*) and
[`diffbot_controllers.yaml`](../ros2_ws/src/vto_description/config/diffbot_controllers.yaml)
(the *controllers* running on top of it).

Prereq: [doc 00 §5](00-ros2-robot-stack-overview.md) (the muscle layer) and
[doc 01](01-urdf-the-robots-body.md) (links/joints).

---

## 1. The problem ros2_control solves

You have a planner that says *"move at v = 0.5 m/s, ω = 0.3 rad/s."* You have two
motors. Something must:

1. convert `(v, ω)` → left/right wheel speeds (**kinematics**),
2. send those speeds to the motors (**hardware**),
3. read encoders back and compute *"how far have I actually gone?"* (**odometry**),
4. do all of this at a **fixed, reliable rate** (100 Hz here).

That "something" is **ros2_control**. Crucially, it's designed so the *same* code
runs in Gazebo and on a real robot — you swap only the bottom layer.

---

## 2. The two halves (and how they meet)

```
        ┌─────────────────────────────────────────────┐
        │ CONTROLLERS   (diffbot_controllers.yaml)     │  the "brain reflexes"
        │  • diff_drive_controller                     │
        │  • joint_state_broadcaster                   │
        └───────────────┬─────────────────────────────┘
                        │  command_interfaces (write velocity)
                        │  state_interfaces   (read pos/vel)
        ┌───────────────▼─────────────────────────────┐
        │ HARDWARE INTERFACE  (diffbot.ros2_control.xacro)│  the "nerves"
        │  • plugin: gz_ros2_control/GazeboSimSystem    │  (sim)
        │    …or a real hardware plugin on a robot       │
        └───────────────┬─────────────────────────────┘
                        ▼
                 motors / Gazebo joints
```

- **Hardware interface** = the *nerve endings*: it exposes named
  `command_interface`s (things you can set) and `state_interface`s (things you can
  read) for each joint. Defined in the **URDF** (the xacro file).
- **Controllers** = the *reflexes*: plugins that read/write those interfaces to do
  a job. Configured in the **YAML**.
- The **`controller_manager`** is the host process that loads the hardware
  interface, loads controllers, and ticks everything at `update_rate`.

> Mental model: the URDF says *what wires exist*; the YAML says *what programs run
> on those wires*.

---

## 3. The hardware interface (the xacro file)

```xml
<ros2_control name="DiffbotSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-20.0</param>
      <param name="max">20.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  ...
</ros2_control>
```

Line by line:

- **`<ros2_control type="system">`** — declares a hardware component. `system` =
  a multi-joint robot managed as one unit (vs `sensor` or `actuator` for
  single-purpose components).
- **`<plugin>gz_ros2_control/GazeboSimSystem</plugin>`** — *the swap point.* This
  says "the hardware is Gazebo." On a real robot you'd replace this **one line**
  with your motor driver's plugin (e.g. `my_robot/MotorHardware`) — controllers
  and everything above stay identical. **This is the payoff of ros2_control.**
- For each wheel joint (matching the joint names from the URDF, doc 01 §5):
  - **`command_interface name="velocity"`** — "you may command this wheel's
    *velocity*" (rad/s), clamped to ±20. We command velocity, not position or
    torque, because diff-drive controllers think in wheel speeds.
  - **`state_interface position` / `velocity`** — "you may read this wheel's angle
    and speed." Position (accumulated wheel angle from the encoder) is what
    odometry integrates to know how far we've driven.

So this file declares: *two velocity-commandable wheels, each reporting position +
velocity, backed by Gazebo.*

---

## 4. The controllers (the YAML)

### 4a. The manager (lines 1–9)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
```

- **`update_rate: 100`** — the whole control loop ticks at **100 Hz** (every
  10 ms). Deterministic timing is why control lives here and not in a random node.
- Then it registers **two controllers** by name + plugin `type`:
  - **`joint_state_broadcaster`** — *reads* every joint's state and publishes
    `/joint_states`. This is what feeds `robot_state_publisher` so the **wheels
    turn in TF/rviz** (doc 01 §0). It's read-only — a "broadcaster," not a
    "controller."
  - **`diff_drive_controller`** — the real actor (next).

### 4b. The diff_drive_controller (lines 11–33)

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.34
    wheel_radius: 0.08
    odom_frame_id: odom
    base_frame_id: base_footprint
    enable_odom_tf: true
    ...
```

- **`left/right_wheel_names`** — which joints (from the hardware interface) this
  controller drives. The names *must* match the URDF joints exactly.
- **`wheel_separation: 0.34`, `wheel_radius: 0.08`** — ⚠️ **these must equal the
  URDF geometry** (doc 01 §2). The controller uses them for the kinematics below;
  a mismatch means the robot's *reported* motion diverges from its *actual* motion
  → odometry drift. This is the single most common diff-drive bug.
- **`odom_frame_id` / `base_frame_id` / `enable_odom_tf: true`** — the controller
  publishes the **`odom → base_footprint`** transform (doc 00a!). *This is where
  that TF edge is born.* The comment on line 19 notes we'll hand this job to
  `robot_localization` in Phase 2 (then set this to `false` to avoid two
  publishers fighting over one edge — the "teleport" bug from doc 00a Q4).
- **`open_loop: false`** — compute odometry from *encoder feedback*, not from the
  commanded velocity (more accurate).
- **`use_stamped_vel: false`** — accept a plain `geometry_msgs/Twist` on `cmd_vel`.
- **kinematic limits** (lines 28–33) — cap velocity and *acceleration* so the
  robot can't jerk. Acceleration limits give smooth, realistic motion.

---

## 5. The kinematics: the actual math

This is the heart of "differential drive," and a guaranteed interview question.
The controller converts body velocity `(v, ω)` into wheel angular speeds
`(ω_L, ω_R)`.

Let `r` = wheel radius (0.08 m), `L` = wheel separation (0.34 m).

**Inverse kinematics** (command → wheels), what `diff_drive_controller` runs every
tick:

```
ω_L = (v − ω·L/2) / r
ω_R = (v + ω·L/2) / r
```

Read it physically:
- Pure forward (`ω = 0`): both wheels spin at `v/r`. Straight. ✓
- Pure spin (`v = 0`): wheels spin *opposite* (`∓ ωL/2r`). Rotate in place. ✓
- Turning left (`ω > 0`): right wheel faster than left. ✓

**Forward kinematics** (wheels → motion), what odometry uses to integrate pose:

```
v = r·(ω_R + ω_L) / 2          # average of the two wheels
ω = r·(ω_R − ω_L) / L          # difference, scaled by separation
```

Then odometry integrates over each timestep `dt`:

```
θ  += ω·dt
x  += v·cos(θ)·dt
y  += v·sin(θ)·dt
```

…producing the `odom → base_footprint` pose. Notice the **non-holonomic**
structure: there's **no sideways term** — the robot can only move along its
heading `θ` and rotate. It *cannot* translate sideways. That constraint is exactly
what makes trajectory planning for these robots interesting (and is why your
car-like/snake models matter).

> **Why does the controller need `wheel_radius` and `wheel_separation`?** They're
> `r` and `L` in every equation above. Wrong values → wrong `(v, ω)` → drifting
> odometry. Now the "must match the URDF" warning makes mechanical sense.

---

## 6. End-to-end data flow

```
  teleop / planner
        │  geometry_msgs/Twist  (v, ω)
        ▼
   /diff_drive_controller/cmd_vel_unstamped
        │
        ▼
  diff_drive_controller ──(inverse kinematics)──► velocity command_interface
        │                                              │
        │                                              ▼
        │                                     gz_ros2_control  → Gazebo wheel joints
        │                                              │
        │  ◄──── state_interface (position, velocity) ─┘   (encoders)
        ▼
  (forward kinematics + integration)
        ├──► /odom            (nav_msgs/Odometry)
        └──► /tf  odom→base_footprint

  joint_state_broadcaster ──► /joint_states ──► robot_state_publisher ──► wheel TF
```

Two outputs feed the rest of the stack: **`/odom`** (where am I — feeds
localization in Phase 2) and **`/joint_states`** (wheel angles — makes the model
animate in rviz).

---

## 7. How it starts up (the launch wiring)

From [`gazebo.launch.py`](../ros2_ws/src/vto_simulation/launch/gazebo.launch.py),
controllers are **spawned** after the robot exists:

```python
jsb  = Node(package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster"])
diff = Node(package="controller_manager", executable="spawner",
            arguments=["diff_drive_controller"])
# only after the robot has spawned in Gazebo:
RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, diff]))
```

The **`spawner`** loads a controller into the running `controller_manager` and
activates it. We gate it on the robot having spawned, because the hardware
interface (Gazebo joints) must exist first. (We'll dissect this whole launch file
in doc 03.)

---

## Run it

Inside the container (`source install/setup.bash`):

```bash
ros2 launch vto_bringup sim.launch.py           # bring up robot + controllers

# in another shell (docker exec -it vehtrajopt bash; source install/setup.bash)
ros2 control list_controllers                   # both should be "active"
ros2 topic echo /joint_states --once            # wheels reporting?
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.4}, angular: {z: 0.3}}}"   # drive it
ros2 topic echo /odom --field pose.pose.position     # pose changing?
```

Automated: `bash scripts/phase0_smoke_test.sh`. Full list in the
[commands cheatsheet](commands.md).

---

## Interview angle

**Q: What are the two halves of ros2_control?**
The *hardware interface* (exposes `command_interface`s and `state_interface`s per
joint; defined in URDF) and the *controllers* (plugins that read/write those
interfaces; configured in YAML). The `controller_manager` hosts both at a fixed
rate.

**Q: Why is ros2_control worth the complexity vs. publishing motor commands?**
Hardware abstraction: the same controllers run in sim and on hardware by swapping
one `<plugin>` line. Plus deterministic timing, standard lifecycle, safety limits,
and reusable controllers.

**Q: Derive differential-drive inverse kinematics.**
`ω_L = (v − ωL/2)/r`, `ω_R = (v + ωL/2)/r`, where `r` = wheel radius, `L` = wheel
separation. Forward: `v = r(ω_R+ω_L)/2`, `ω = r(ω_R−ω_L)/L`.

**Q: What makes a diff-drive robot non-holonomic?**
Its odometry has no lateral term — it can only move along its heading and rotate,
not translate sideways. Fewer controllable DOF (2: v, ω) than configuration DOF
(3: x, y, θ).

**Q: Where does the `odom→base_link` transform come from?**
The odometry source — here `diff_drive_controller` with `enable_odom_tf: true`. In
Phase 2 `robot_localization` takes over and the controller's TF is disabled so two
nodes don't publish the same edge.

**Q: What does `joint_state_broadcaster` do?**
Reads all joint states and publishes `/joint_states`; `robot_state_publisher`
consumes it to update the movable TF frames (spinning wheels) — it doesn't command
anything.

**Q: Why command wheel *velocity* rather than position or torque?**
Diff-drive kinematics are naturally expressed in wheel angular velocity; velocity
control gives smooth speed regulation. (Position→jerky, torque→needs a dynamics
model, which is Phase 1 MPC territory.)

---

## Where this maps in our repo

- Hardware interface:
  [`diffbot.ros2_control.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.ros2_control.xacro)
- Controllers:
  [`diffbot_controllers.yaml`](../ros2_ws/src/vto_description/config/diffbot_controllers.yaml)
- Loaded by the gz plugin (doc 03) and spawned in
  [`gazebo.launch.py`](../ros2_ws/src/vto_simulation/launch/gazebo.launch.py)
- **Next (doc 03):** Gazebo simulation — the `gz_ros2_control` plugin, the world,
  spawning, the `ros_gz` bridge, and `use_sim_time`.
