# 16 — Navigation pipeline: node/topic/TF diagram

Static reference for the full stack as wired today: `maze_astar.launch.py`
(Gazebo → localization → planning → control), plus where `vto_bench` taps in.

---

## 1. Why a diagram, and why *this kind*

A ROS 2 system isn't one program — it's a graph of independent processes (nodes)
that only agree on message contracts (topics/services/actions) and coordinate
frames (TF). None of that is visible by reading any single file: `astar_planner_node.py`
doesn't know pure_pursuit exists, it just publishes `/plan` and trusts *something*
subscribes. Understanding the system means understanding the **graph**, not any
one node — which is exactly why "draw the architecture" is a standard interview
ask: it tests whether you can reason about a distributed system, not just write a
node.

Where this sits conceptually: this diagram *is* the perception→planning→control
pipeline made concrete — `/scan`, `/odom`, `/imu` (perception input) →
`ekf_node` + AMCL (**two-stage** state estimation — local fusion, then global
correction, see docs/17 §2 and the Interview angle below) → `/plan` (planning)
→ `/cmd_vel` (control) → Gazebo physics → `/joint_states` back around the loop.

> **Status:** `/imu` (doc 17) and `ekf_node` (doc 18) are both live in
> `maze_astar.launch.py`. The wiring below matches the real launch file.

## 2. Method: static annotated diagram vs. the alternatives

Three ways to get "a picture of the graph":

| Method | What it shows | Limitation here |
|---|---|---|
| `rqt_graph` | **Live** node↔topic graph, auto-discovered | Requires the stack running; doesn't label message *types*; doesn't show TF, services, or lifecycle transitions; redraws differently depending on what's active (e.g. which controller) — not a stable reference to re-read later |
| `tf2_tools view_frames` | **Live** TF tree only, with publish rates | Only TF — no topics, no message types, no services |
| Static annotated diagram (this doc) | Everything: topics **with types and QoS notes**, services, TF, and the parts that *aren't* wired (e.g. no Nav2 actions yet) | Goes stale if the code changes — has to be kept up to date by hand |

**Choice: static annotated diagram, in the repo, alongside the other numbered docs.**
Reasons specific to this project: (1) I want it to double as interview prep material —
`rqt_graph`'s screenshot answers "what's running" but not "what type is that message"
or "why is there no action server here"; (2) it needs to be readable *without* booting
the Gazebo+AMCL stack; (3) it should show what's **absent** (Nav2's BT navigator /
`NavigateToPose` action isn't in this pipeline yet — goals go in over a bare topic),
which a live introspection tool can't tell you since it only shows what exists.

This doesn't replace `rqt_graph`/`view_frames` — those are still the tool for
"is the *live* system wired the way this diagram says." Use both.

---

## 3. Node / topic graph — the overall pipeline

One picture: `gz sim`, `amcl`, `map_server`, `planner` (`astar_planner`),
`controller` (`pure_pursuit` | `mpc_controller` | `mppi_controller` — one at a
time), and `rviz2` / `benchmark_node` as the two ends of who talks to whom.
Every arrow is a topic (or TF, marked) actually wired in `maze_astar.launch.py`.

/odom: is a topic (nav_msgs/Odometry) published by `diff_drive_controller` — raw wheel odometry, still drift-prone. Once `ekf_node` exists it is consumed *only* by `ekf_node`, not directly by amcl or the controllers anymore (see below).
/imu: is a topic (sensor_msgs/Imu) bridged from Gazebo's IMU sensor (doc 17) — also consumed only by `ekf_node`.
/odometry/filtered: is a topic (nav_msgs/Odometry) published by `ekf_node` — the fused pose+twist estimate, which becomes what amcl's motion model and the controllers actually read (the fused twist is materially better for controllers: less noisy heading-rate feedback than raw wheel odom, doc 17 §1).
`ekf_node` also takes over broadcasting `/tf:odom→base_footprint` from `diff_drive_controller` (`enable_odom_tf` gets set to `false` — the switch already flagged in [diffbot_controllers.yaml:19](../ros2_ws/src/vto_description/config/diffbot_controllers.yaml#L19)). This is the one TF ownership change in the whole tree.
TransformListener: is a ROS 2 built-in node that subscribes to the TF tree and provides a service to look up transforms between frames, used by planners and controllers to get the robot's pose in different frames.

```
                                    ———————————————————————
                                   |     gz sim (100Hz)    |
    /————————————————————————————> |(diff_drive_controller)|
    |     /cmd_vel                  ———————————————————————
    |                                  |            |
    |                             /imu |            | /scan
    |                            /odom v            | 
    |   /odometry/filtered   ————————————————————   | 
    | /———————————————————— |     ekf_node       |  | 
    | |                     |(robot_localization)|  | 
    | |                      ————————————————————   |
    | |          /odometry/filtered  |              |
    | |      /tf:odom→base_footprint |              |
    | |                              v              v
    | |     —————————————  /map    ——————————————————————
    | |    |  map_server |——————> |        amcl          |
    | |    |             |        |robot_state_publisher |
    | |     —————————————          ——————————————————————     
    | |           |                        | /amcl_pose,
    | |           | /map                   | /particle_cloud
    | |           v                        v /tf:map→odom               
    | |  —————————————————   /plan     ————————————————————
    | | |     planners    |—————————> |   rviz2 / bench    |
    | | |      (A*)       |<————————— |       (20Hz)       |
    | | |TransformListener|/goal_pose |                    |
    | |  —————————————————             ————————————————————
    | |            |                                   ^
    | |      /plan |                                   |
    | |            v                                   |
    | |    ——————————————————                          |
    | |   | controllers(20Hz) |————————————————————————/
    | |   |(Pure Pursuit/     |  /controller/compute_ms
    | |   |  MPC/MPPI)        |  
    | |   |TransformListener()|
    | |   | (get robot pose)  |
    | |    ——————————————————
    | |      ^           | 
    | |      |(get vel.) | /cmd_vel   
    | \——————/           |  
    \————————————————————/
      /cmd_vel
```

**What's *not* in this graph:** no ROS 2 *actions* anywhere — goals go in as a
bare `/goal_pose` topic, not a `NavigateToPose` action (that's what Nav2's BT
navigator would add, and it isn't wired in yet). `lifecycle_manager`'s
`ChangeState`/`GetState` **services** (not shown above — they're one-shot
request/response, not a data-flow edge) sequence `map_server` and `amcl`
through their `LifecycleNode` state machine at startup: `map_server` must be
*active* before `amcl` configures against it (docs/09).

**Notes on what's drawn:**
- `gz sim`'s box folds in `controller_manager` and `ros_gz_bridge` — neither is
  a separate node worth its own box for this picture: `controller_manager` is
  hosted **inside the Gazebo process** by the `gz_ros2_control::GazeboSimROS2ControlPlugin`
  ([diffbot.gazebo.xacro:5](../ros2_ws/src/vto_description/urdf/mobile/diffbot.gazebo.xacro#L5)),
  and `ros_gz_bridge` only relays `/scan` and `/imu` (`/cmd_vel`/`/odom` are
  already native ROS topics off the plugin, not bridged — see
  [bridge.yaml](../ros2_ws/src/vto_simulation/config/bridge.yaml)).
- `ekf_node` (`robot_localization`) is its own standalone process — unlike
  `controller_manager` it isn't hosted inside anything else, since it's plain
  ROS consuming plain ROS topics/messages, the same node unmodified whether
  `/odom`+`/imu` came from Gazebo or real hardware (doc 17 §3, last Q&A).
- `TF odom→base_footprint` is now broadcast by `ekf_node`, **not**
  `diff_drive_controller` (which stops once `enable_odom_tf: false` lands) —
  it bypasses `amcl`'s topic-level inputs entirely and is the reason `amcl`'s
  arrow is drawn straight into it rather than through `map_server`.
- Controllers' `/odom` subscription is redirected to `ekf_node`'s
  `/odometry/filtered` — not drawn as a full long arrow (it would cross
  `map_server`/`amcl`/`planners`) the same way the composed `map→base_footprint`
  TF lookup isn't drawn either; both are noted inline/in prose instead of as a
  picture edge, to keep the diagram legible.
- `controller` is one node at a time (`controller:=pursuit|mpc|mppi` in
  [maze_astar.launch.py](../ros2_ws/src/vto_bringup/launch/maze_astar.launch.py)) —
  drawn as one box since all three share the identical `/plan,/odom → /cmd_vel`
  interface (the point of docs/13 §"drop-in").
- `rviz2`/`benchmark_node` sit **outside** the control loop — they only observe
  it (`/amcl_pose`, `/particle_cloud`, `/plan`, `/controller/compute_ms`) and
  inject the one topic that actually starts a planning cycle: `/goal_pose`
  (RViz's "2D Nav Goal" click, or `benchmark_node`'s tour driver).

## 4. TF tree

```
|———————|  amcl: dynamic    |————————|  ekf_node:         |—————————————————|
|  map  | ————————————————> |  odom  | —————————————————> | base_footprint  |
|———————|  correction       |————————|  dynamic (fused,   |—————————————————|
                                        odom+imu)                 |
                                                                  | 
                                /—————————————————————————————————/  
                                |          robot_state_publisher:
                                v           fixed (URDF)
                         |——————————————|
                         |  base_link   |
                         |——————————————|
                ———————————————|———————————————|————————————
               |               |               |            |
   dynamic     |    dynamic    |        fixed  |      fixed |
(/joint_states)|(/joint_states)| (URDF, at CoM)|     (URDF) |
               v               v               v            v
    |—————————————|  |——————————————| |—————————————|  |—————————————|
    | wheel_left  |  | wheel_right  | |  imu_link   |  | lidar_link  |
    |—————————————|  |——————————————| |—————————————|  |—————————————|
```

- **`map→odom`** is the one AMCL owns and periodically corrects (docs/11, docs/12)
  — this is the transform the spike-bug investigation (docs/14 §6) needs to log
  *in isolation* to tell "AMCL corrected the estimate" apart from "the robot
  physically moved."
- **`odom→base_footprint`** is continuous, high-rate, and — once `ekf_node`
  exists — comes from the **fused** estimate (wheel odom + gyro), not raw
  wheel odometry alone. It still accumulates *drift* (not jumps) between AMCL
  corrections; fusion slows the drift rate, it doesn't eliminate it (no
  absolute reference — that's still AMCL's job).
- Everything from `base_link` down is the URDF-defined body — fixed joints
  (chassis→lidar, chassis→imu) publish once (latched-ish, static by
  convention) and continuous joints (wheels) update at `/joint_states` rate
  via `joint_state_broadcaster`. `imu_link` sits at `base_link`'s own origin
  (not offset, unlike `lidar_link`) — doc 17 §3 explains why: off-center
  placement would leak lever-arm/centripetal acceleration into the reading.
- `astar_planner` and each controller read `map→base_footprint` (the **composed**
  transform, i.e. `map→odom` ∘ `odom→base_footprint`) via `tf2_ros.Buffer.lookup_transform`
  — they don't care about the intermediate `odom` frame at all, which is exactly
  why the benchmark's current per-tick logging (only the composed pose) can't
  separate the two contributions.

---

## Interview angle

- **Q: Why does AMCL publish `map→odom` instead of `map→base_footprint` directly?**
  Because `odom→base_footprint` already exists (owned by the wheel-odometry
  source) and is continuous/high-rate; if AMCL also owned the base frame directly,
  two nodes would be broadcasting conflicting transforms for the same frame. By
  only correcting the *offset* between `odom` and `map`, AMCL can publish at its
  own (lower, particle-filter-limited) rate while `odom→base_footprint` keeps
  updating at full wheel-odometry rate — `lookup_transform` composes them for you.
- **Q: What's a lifecycle node, and why do `map_server`/`amcl` use one but
  `astar_planner` doesn't?** A `LifecycleNode` has an explicit state machine
  (unconfigured → inactive → active) driven by a manager over services, so startup
  order is deterministic (map_server must be *active* — i.e. actually serving
  `/map` — before amcl configures against it). Plain nodes start publishing/
  subscribing the instant they spin up, with no cross-node ordering guarantee —
  fine for `astar_planner`, which just waits on its first `/map` message rather
  than needing to be sequenced relative to another node.
- **Q: Why does `ekf_node` sit *between* `gz sim` and `amcl` instead of amcl
  just reading `/odom` and `/imu` directly?** Single responsibility: `ekf_node`
  answers "what's my best local motion estimate" (fusing proprioceptive
  sensors), `amcl` answers "where am I on this map" (correcting that estimate
  against an exteroceptive one, the lidar scan). Folding fusion logic into
  AMCL would couple two different jobs with different real-time/algorithmic
  requirements into one node, and — concretely here — every *other* consumer
  of odometry (the controllers) would have needed to duplicate the fusion or
  read pre-AMCL, worse ergonomics either way.
- **Q: Where would a `NavigateToPose` action fit if you added the Nav2 BT
  navigator later?** It would sit in front of `astar_planner`, replacing the bare
  `/goal_pose` topic: a client sends a `NavigateToPose` goal, the BT navigator
  runs a behavior tree that calls the global planner and controller as **plugins**
  (not raw topics) and streams back feedback/result over the action protocol —
  giving you preemption, feedback, and recovery behaviors that a fire-and-forget
  topic can't.
- **Q: Topic vs. service vs. action — how did the choices in this graph map to
  that distinction?** Continuous/streaming data with no reply needed → topic
  (`/scan`, `/odom`, `/plan`, `/cmd_vel`). One-shot request/response where the
  caller blocks for a result → service (`lifecycle` `ChangeState`/`GetState`).
  Long-running, cancellable, with progress feedback → action (none used yet here;
  that's what `NavigateToPose` would be).
