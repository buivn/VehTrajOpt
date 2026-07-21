# 03 — Gazebo Simulation: Giving the Robot a World

Docs 01–02 built the robot's body and control. But there's nothing for it to
stand on yet. This doc adds **physics + a world + sensors** via Gazebo Harmonic,
and wires it all together. When it works, you get a robot that *drives*. That
completes **Phase 0**.

Files:
[`empty.sdf`](../ros2_ws/src/vto_simulation/worlds/empty.sdf) (the world),
[`diffbot.gazebo.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro)
(sim plugins on the robot),
[`bridge.yaml`](../ros2_ws/src/vto_simulation/config/bridge.yaml) (the ROS↔Gazebo
bridge), and
[`gazebo.launch.py`](../ros2_ws/src/vto_simulation/launch/gazebo.launch.py) (the
conductor).

Prereq: [doc 00 §6](00-ros2-robot-stack-overview.md) (sim), plus docs 01–02.

---

## 1. Two separate worlds that must be bridged

The key mental model: **ROS 2 and Gazebo are two independent systems** that each
have their own messages, their own clock, and their own transport. They don't
automatically talk.

```
   ┌──────────────┐         bridge          ┌──────────────┐
   │    ROS 2     │  ◄───────────────────►  │    Gazebo    │
   │ nodes/topics │   ros_gz_bridge         │  gz-transport│
   │  (rclcpp)    │                         │   physics    │
   └──────────────┘                         └──────────────┘
```

There are **two ways** data crosses that boundary — and it's important to know
which is which:

1. **The `gz_ros2_control` plugin** — lives *inside* the robot model in Gazebo and
   speaks ROS 2 directly. It handles `/cmd_vel`, `/odom`, `/joint_states`, TF.
2. **The `ros_gz_bridge`** — a general translator for *everything else* (clock,
   LiDAR, camera, IMU).

That's why our [`bridge.yaml`](../ros2_ws/src/vto_simulation/config/bridge.yaml) is
almost empty — the control-related topics are *already* handled by the plugin, so
the bridge only needs `/clock` (for now).

---

## 2. The world: `empty.sdf`

**SDF** (Simulation Description Format) is Gazebo's native format — like URDF but
for *whole worlds* (robots + environment + physics + lighting). Our world is
minimal:

```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>      <!-- 1 ms physics step = 1000 Hz -->
  <real_time_factor>1.0</real_time_factor>  <!-- run at wall-clock speed -->
</physics>

<plugin filename="gz-sim-physics-system"          .../>  <!-- integrates dynamics -->
<plugin filename="gz-sim-user-commands-system"     .../>  <!-- spawn/delete/teleport -->
<plugin filename="gz-sim-scene-broadcaster-system" .../>  <!-- streams state to the GUI -->
<plugin filename="gz-sim-sensors-system"           .../>  <!-- renders LiDAR/camera (Phase 2) -->

<light type="directional" name="sun"> ... </light>
<model name="ground_plane"> ...static plane... </model>
```

Notes:

- **Gazebo is plugin-based too.** Even core features (physics, sensors) are
  plugins loaded into the world. The **`sensors-system`** with `ogre2` rendering
  is here now so Phase 2's LiDAR/camera work with no world changes.
- **`max_step_size = 0.001`** → physics runs at **1000 Hz**, finer than our
  control loop (100 Hz). Physics should always be *at least* as fast as control,
  or the simulation is unstable.
- **`ground_plane`** is `static` (infinite mass, never moves) — the floor.
- **This is "Gazebo Harmonic"** (aka `gz-sim`, formerly "Ignition"), the modern
  Gazebo — *not* the deprecated "Gazebo Classic." They have different plugin names
  and formats; mixing them up is a common beginner trap.

---

## 3. Sim plugins on the robot: `diffbot.gazebo.xacro`

This is the third URDF fragment (doc 01 §8). It adds Gazebo-specific things the
plain URDF can't express.

### 3a. The control plugin (lines 4–9) — *the crucial link*

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find vto_description)/config/diffbot_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**This is what actually launches the `controller_manager` inside Gazebo.** It:

- reads the `<ros2_control>` block from the URDF (doc 02 §3) → knows the hardware
  interface,
- loads `diffbot_controllers.yaml` (doc 02 §4) → knows which controllers to run,
- ticks them at the YAML's `update_rate` (100 Hz), backed by Gazebo's physics.

So the chain from doc 02 finally closes: **URDF declares interfaces → this plugin
hosts them → YAML controllers drive them → Gazebo joints move.** Without this
plugin, the `<ros2_control>` block in the URDF is inert.

### 3b. Friction (lines 11–20)

```xml
<gazebo reference="left_wheel">  <mu1>1.0</mu1> <mu2>1.0</mu2> </gazebo>
...
<gazebo reference="caster">      <mu1>0.0</mu1> <mu2>0.0</mu2> </gazebo>
```

`mu1`/`mu2` are the two friction coefficients (along/across the contact). Physics
needs friction that the *visual* URDF has no concept of:

- **Wheels: `mu = 1.0`** — grip, so spinning them actually propels the robot.
  Zero friction = wheels spin, robot goes nowhere.
- **Caster: `mu = 0.0`** — frictionless, so it slides freely instead of fighting
  the drive wheels (doc 01 §7). This is the standard sim shortcut for a ball
  caster.

`<gazebo reference="LINK">` attaches sim properties to a link *defined elsewhere*
(in the body xacro) — that's how we keep physics data out of the clean body file.

---

## 4. The bridge: `bridge.yaml`

```yaml
- ros_topic_name: "/clock"
  gz_topic_name:  "/clock"
  ros_type_name:  "rosgraph_msgs/msg/Clock"
  gz_type_name:   "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

The bridge translates message *types* across the ROS/Gazebo boundary. Each entry
maps one topic: its name on each side, the type on each side, and the direction.

Right now we bridge exactly one topic: **`/clock`**, `GZ_TO_ROS`. Why it matters:

### `use_sim_time` — the clock everyone must share

In simulation, "time" is **Gazebo's** time, not the wall clock (if physics runs at
0.5× real-time, one sim-second takes two real-seconds). Every ROS node must agree
on this, or timestamps and TF lookups break.

- Gazebo publishes simulation time on **`/clock`**.
- The bridge forwards it into ROS.
- Every node runs with the parameter **`use_sim_time: true`** so it reads time from
  `/clock` instead of the system clock.

You'll see `use_sim_time` set throughout our launch files. Forgetting it is a
classic bug: TF complains about timestamps "extrapolation into the future/past."

> In Phase 2, this same file gains LiDAR/camera/IMU entries (`GZ_TO_ROS`) to pull
> sensor data into ROS for fusion. The pattern is identical.

---

## 5. The launch file: `gazebo.launch.py` — the conductor

This ties **everything from docs 01–03** into one command. A ROS 2 launch file is
Python that returns a list of things to start. Ours starts five, in a deliberate
order.

```python
# 1. robot_description + robot_state_publisher (reuses description.launch.py)
description = IncludeLaunchDescription(... "description.launch.py",
                                       launch_arguments={"use_sim": "true"})

# 2. Gazebo itself (server + GUI), loading our world
gz_sim = IncludeLaunchDescription(... "gz_sim.launch.py",
                                   launch_arguments={"gz_args": ["-r -v4 ", world]})

# 3. spawn the robot INTO Gazebo from the /robot_description topic
spawn = Node("ros_gz_sim", "create",
             arguments=["-topic", "robot_description", "-name", "diffbot", "-z", "0.15"])

# 4. the ROS↔Gazebo bridge (/clock)
bridge = Node("ros_gz_bridge", "parameter_bridge", parameters=[{"config_file": bridge_cfg}])

# 5. spawn controllers — but ONLY after the robot exists
jsb  = Node("controller_manager", "spawner", arguments=["joint_state_broadcaster"])
diff = Node("controller_manager", "spawner", arguments=["diff_drive_controller"])
RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, diff]))
```

The pieces that matter:

- **`IncludeLaunchDescription`** — launch files *compose*. We reuse
  `description.launch.py` (doc 01) instead of duplicating `robot_state_publisher`.
  This is why `vto_bringup`'s `sim.launch.py` can then include *this* file — layers
  of composition.
- **`gz_args="-r -v4"`** — `-r` runs physics immediately (don't start paused);
  `-v4` = verbose logging. `world` is our `empty.sdf`.
- **Spawn from a topic (line 35):** `create -topic robot_description` tells Gazebo
  "grab the robot from the `/robot_description` topic that `robot_state_publisher`
  is publishing." So the **same URDF** is the single source of truth for TF *and*
  the physics body. `-z 0.15` drops it 15 cm above ground so it settles, not
  clips.
- **Ordering via events (lines 49–50):** the `spawner`s must run *after* the robot
  (and its `gz_ros2_control` plugin) exist, or there's no `controller_manager` to
  load into. `RegisterEventHandler(OnProcessExit(target_action=spawn, ...))` means
  "when the `spawn` process finishes, *then* start the controllers." This kind of
  **event-driven ordering** is how real launch files avoid race conditions.

### The startup sequence, in order

```
robot_state_publisher  ──►  publishes /robot_description
        │
Gazebo starts, loads empty.sdf world
        │
create -topic robot_description  ──►  robot appears in Gazebo,
        │                             gz_ros2_control plugin starts controller_manager
        │  (spawn process exits)
        ▼
OnProcessExit fires  ──►  spawn joint_state_broadcaster + diff_drive_controller
        │
        ▼
   robot is drivable:  /cmd_vel → wheels,  /odom + TF flowing
```

---

## 6. Running Phase 0 (in the Docker container)

```bash
# top-level: Gazebo + robot + controllers + rviz
ros2 launch vto_bringup sim.launch.py

# drive it (new shell; source install/setup.bash first)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

**Success = Phase 0 done:** robot moves in Gazebo, model + TF animate in rviz,
`ros2 topic echo /odom` shows pose changing. If so, the entire sense→…→act stack's
*act* half is working end-to-end.

### If something's off (quick triage)

- `ros2 control list_controllers` → are both controllers `active`?
- `ros2 topic hz /clock` → is sim time flowing (bridge alive)?
- `ros2 topic echo /joint_states` → are wheels reporting? (broadcaster alive)
- rviz "extrapolation" errors → a node is missing `use_sim_time: true`.

---

## 7. Sim vs real — what the plugin and bridge *actually* are

Three misconceptions worth killing early. They come straight from asking "so what
is this `<plugin>` line, really?"

### 7a. `GazeboSimSystem` is **not** a robot model — it's generic glue

```xml
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
```

`gz_ros2_control/GazeboSimSystem` is a **generic hardware-interface plugin**, not a
diff-drive robot. It knows nothing about diff-drive. It's robot-agnostic and does
exactly three things:

- reads the `<ros2_control>` joint list from **your** URDF (doc 02 §3),
- connects each declared joint to the matching joint in Gazebo's physics,
- implements the `SystemInterface` `read()`/`write()` contract so ros2_control can
  drive Gazebo.

**Your `diffbot.urdf.xacro` *is* the robot model — you authored it.** The same
`GazeboSimSystem` plugin serves a car, an arm, or a snake; only the URDF changes.
And the *diff-drive behavior* comes entirely from the **`diff_drive_controller`**
(the controller half), never from Gazebo.

```
   your URDF              = the robot model               (you author)
   GazeboSimSystem        = generic ros2_control↔Gazebo adapter  (you reference)
   diff_drive_controller  = the diff-drive brain          (you configure)
```

> **Alternative worth knowing:** Gazebo ships a *native* `gz::sim::systems::DiffDrive`
> system plugin — a self-contained diff-drive that subscribes to a velocity command
> and moves the model, **without ros2_control**. We deliberately did **not** use it
> because it only works in Gazebo (you'd throw it away on hardware). Our
> `GazeboSimSystem` + `diff_drive_controller` path runs *unchanged* on a real robot
> (see 7c). Two valid approaches; we picked the portable one.

### 7b. The bridge is a **separate node we run**, not automatic

Two facts:

1. **Gazebo already has its own topics**, independent of ROS — it runs on
   **`gz-transport`**, its own middleware (e.g. `/world/empty/model/diffbot/...`).
   These are *not* ROS topics.
2. **`ros_gz_bridge` is a node we explicitly launch** (doc 03 §5) and configure in
   `bridge.yaml`. Gazebo does **not** auto-create ROS topics. The bridge is a
   translator process that, per config entry, creates a ROS-side counterpart and
   copies messages across.

```
  gz-transport topic  ◄──[ ros_gz_bridge node ]──►  ROS 2 topic
      (Gazebo)            translates the type            (rclcpp)
```

The usual direction is the *reverse* of "declare a ROS topic and Gazebo builds it":
a Gazebo plugin publishes on a **gz** topic, and you add a bridge entry to *expose*
it as a ROS topic:

```yaml
# Phase 2 LiDAR example:
- gz_topic_name:  "/lidar"      # Gazebo already publishes this
  ros_topic_name: "/scan"       # bridge creates this ROS topic
  direction: GZ_TO_ROS
```

So: **you declare a mapping between an existing gz topic and a ROS topic; the
bridge creates the counterpart.** It's a configured translator, not an
auto-generator. (And control topics skip the bridge entirely — `gz_ros2_control`
links `rclcpp` inside Gazebo and publishes them as real ROS topics directly, which
is why `bridge.yaml` only needs `/clock`.)

### 7c. On a real robot, **everything is ROS topics**

There is no Gazebo, no gz-transport, no bridge on hardware. `/cmd_vel`, `/odom`,
`/joint_states`, `/scan` are all plain ROS 2 topics. The `diff_drive_controller`
subscribes `/cmd_vel` and publishes `/odom` *identically*. Only a few boxes change:

| | Simulation | Real robot |
|---|---|---|
| Hardware interface plugin | `gz_ros2_control/GazeboSimSystem` | `your_pkg/MotorHardware` (you write it) |
| Sensor data source | Gazebo sensor plugin **+ bridge** | a **ROS driver node** (vendor's driver) |
| Everything else | — | **identical** |

```
SIM:   /cmd_vel → diff_drive_controller → GazeboSimSystem → Gazebo joints
                                                      ↑ swap only this box
REAL:  /cmd_vel → diff_drive_controller → MotorHardware → real motors
```

Controllers, topics, TF, Nav2, your future MPC — all above the swap line, all
unchanged. That single-box swap is *the* reason ros2_control's complexity is worth
it. (Sensors differ slightly: sim = plugin+bridge, real = driver node — but
downstream everything sees the same `/scan`, `/imu`, so fusion and Nav2 don't care
which world they're in.)

---

## Interview angle

**Q: Is `GazeboSimSystem` a diff-drive robot you load?**
No — it's a *generic* hardware-interface plugin that binds your URDF's declared
joints to Gazebo's physics. The robot model is your URDF; the diff-drive behavior
is the `diff_drive_controller`. The same plugin works for any robot.

**Q: Does Gazebo automatically expose its topics to ROS?**
No. Gazebo has its own `gz-transport` topics. `ros_gz_bridge` is a separate node
you run and configure to translate specific topics between gz-transport and ROS 2.

**Q: On real hardware, what changes vs simulation?**
Only the hardware-interface plugin (Gazebo → your motor driver) and how sensor data
enters (Gazebo plugin+bridge → ROS driver node). Controllers, topics, TF, and
navigation are identical.

**Q: How do ROS 2 and Gazebo communicate?**
Two paths: (1) the `gz_ros2_control` plugin inside the robot speaks ROS 2 directly
for control/odom/joint-states/TF; (2) `ros_gz_bridge` translates everything else
(clock, sensors) between gz-transport and ROS 2 topics, per a type-mapped config.

**Q: What is `use_sim_time` and why does it matter?**
It tells nodes to take time from the `/clock` topic (Gazebo's sim time) instead of
the wall clock, so all nodes and TF share one timeline. Without it, sim runs at ≠
real-time break timestamps and TF lookups.

**Q: URDF vs SDF?**
URDF describes a single robot (ROS-native, links/joints). SDF describes whole
simulation worlds (robots + environment + physics + lights), Gazebo-native. Gazebo
can ingest URDF and convert it internally.

**Q: Why spawn the robot from the `robot_description` topic instead of a file?**
So the *same* URDF is the single source of truth for both TF (via
robot_state_publisher) and the physics body — no divergence between what rviz
shows and what Gazebo simulates.

**Q: Why must controllers be spawned after the robot?**
The `controller_manager` lives inside the robot's `gz_ros2_control` plugin, which
only exists once the robot is spawned. We use a launch event handler
(`OnProcessExit` of the spawn) to enforce that ordering and avoid a race.

**Q: What are `mu1`/`mu2` and why give the caster zero friction?**
Contact friction coefficients. Drive wheels need grip (`mu=1`) to propel; the
caster gets `mu=0` so it slides freely rather than resisting turns — the standard
sim stand-in for a real ball-caster.

**Q: Gazebo Harmonic vs Gazebo Classic?**
Harmonic (`gz-sim`, formerly Ignition) is the modern, actively developed Gazebo
with a plugin-based architecture and different message/plugin naming; Classic is
end-of-life. They're not interchangeable.

---

## Phase 0 — complete ✅

You now have, fully documented:

| Doc | Layer | Result |
|---|---|---|
| 00 / 00a | Concepts + TF | mental model |
| 01 | URDF body | the robot exists as frames |
| 02 | ros2_control | commands → wheels, wheels → odometry |
| 03 | Gazebo | physics + world; **the robot drives** |

**Next: Phase 1 — Control.** From here, code and docs move *together*, one step at
a time. We start with **Pure Pursuit**: finish the stub node
([`pure_pursuit_node.py`](../ros2_ws/src/vto_control/vto_control/pure_pursuit_node.py)),
feed it a path, and watch the robot track it — then document the geometry.
