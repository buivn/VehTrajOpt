# 09 — Integration launch: map_server, lifecycle & the static TF

How `maze_astar.launch.py` brings the whole Phase-1.5 pipeline up in one shot, and
the three ROS 2 concepts it forced us to confront: **lifecycle nodes**, the
**static map→odom transform**, and **why launch order doesn't matter** (latching).

Run it:
```bash
ros2 launch vto_bringup maze_astar.launch.py                # GUI + RViz
ros2 launch vto_bringup maze_astar.launch.py headless:=true # no GUI
```
Then in RViz click **"2D Nav Goal"** and pick a spot; the robot plans and drives.

Headless smoke test (no Gazebo): `ros2_ws/scripts/phase1_5_astar_test.sh`.

---

## 1. What the launch wires

```
 gazebo.launch.py ── maze world + diffbot ── /odom, /joint_states, TF(odom->base)
 map_server ─────── serves maze.yaml ─────── /map (latched)
 lifecycle_manager ─ configure+activate map_server
 static_transform_publisher ─ map->odom (= spawn pose)
 astar_planner ──── /map + /odom + /goal_pose ─► /plan  (odom frame)
 pure_pursuit ───── /plan + /odom ─────────────► /cmd_vel (TwistStamped)
 rviz2 ──────────── displays /map, /plan, TF, robot; publishes /goal_pose
```

Seven nodes, one file. Everything below is *why* three of them are there.

---

## 2. Lifecycle nodes — why map_server needs a babysitter

`map_server` is a **managed (lifecycle) node**. Unlike a plain node that starts
working in its constructor, a lifecycle node boots into the **`unconfigured`** state
and *does nothing* until an external supervisor walks it through a state machine:

```
unconfigured ──configure──► inactive ──activate──► active
```

- **configure**: load params, read the map file, allocate the publisher — but do
  **not** publish yet.
- **activate**: start publishing `/map`.

Why bother? Determinism and orchestration: in a big system (Nav2) you want every
component *configured and verified* before any of them start emitting, and you want
to be able to pause/restart subsystems cleanly. The price is that **nothing happens
until someone triggers the transitions.**

That someone is **`nav2_lifecycle_manager`**. With `autostart: True` and
`node_names: ['map_server']`, it configures then activates the listed nodes in order
at startup. It also keeps a **bond** (a heartbeat) with each; if a managed node dies,
the manager notices and can bring the group down. This is why running `map_server`
*alone* looks broken — no `/map` ever appears. It's waiting to be activated.

> Symptom to remember: "`/map` topic exists but never publishes" → the lifecycle
> node was never activated (no manager, or it's not in `node_names`).

---

## 3. The static map→odom transform — "poor-man's localization"

The TF tree Gazebo/ros2_control gives us is only:
```
odom ──(diff_drive_controller)──► base_footprint ──► wheels, sensors ...
```
There is **no `map` frame in it.** But the occupancy map is stamped `frame_id: map`,
so RViz (Fixed Frame = `map`) can't place the map or the robot until *something*
connects `map` to `odom`.

That connecting transform, **`map→odom`, is normally produced by localization**
(AMCL / robot_localization EKF) and updated continuously to absorb odometry drift.
We don't run localization yet (Phase 2), but we *placed* the robot, so we know the
offset exactly: it's the **spawn pose**. So we publish it as a **static** transform:

```
static_transform_publisher  --x -9.1 --y -14.1  --frame-id map --child-frame-id odom
```

Now the tree is whole (`map→odom→base_footprint→…`), RViz shows everything in the
`map` frame, and a "2D Nav Goal" clicked on the map arrives in the `map` frame.

**The redundancy to be aware of:** the *same* spawn offset now lives in two places —
this static TF (for RViz/TF consumers) and the planner's `spawn_*` params (which it
uses to convert `/plan` into the `odom` frame internally, because pure_pursuit reads
`/plan` with no TF). They must agree. Both are stand-ins that **Phase 2 deletes**:
localization will publish a live `map→odom`, the planner will `lookup_transform`
instead of using constants, and it can then publish `/plan` in `map` and let the
follower transform. One idea, two temporary shortcuts.

---

## 4. Why launch order doesn't matter here (latching + retries)

You might expect we must start `map_server` *before* the planner, or the planner
misses the map. We don't — because of **QoS durability**:

- `/map` is published **TRANSIENT_LOCAL** (latched). Whenever the planner subscribes —
  before *or* after activation — it receives the last map.
- `/goal_pose` and `/odom` are streams/actions; the planner just waits for them, and
  its `_on_goal` guards (`if self.grid is None … ignoring`) handle "goal before map".

So the launch lists nodes in a readable order but relies on **latching + callback
guards**, not strict sequencing. This is the idiomatic ROS 2 way: design for
components arriving in any order, not for a boot script.

---

## Interview angle

- **Q: What is a lifecycle (managed) node and why use one?** A node with an explicit
  state machine (unconfigured→inactive→active). Lets an orchestrator configure and
  verify all components before any start publishing, and restart subsystems cleanly.
  Cost: it does nothing until transitioned.
- **Q: `/map` exists but never publishes — why?** map_server wasn't activated: no
  lifecycle_manager, or autostart off, or it's absent from `node_names`.
- **Q: Difference between `map` and `odom` frames, and who publishes `map→odom`?**
  `odom` is smooth but drifts; `map` is drift-free but can jump on correction.
  Localization (AMCL/EKF) publishes `map→odom`; `odom→base_link` comes from wheel
  odometry. (Here we fake `map→odom` with a static transform = spawn pose.)
- **Q: Why can you launch nodes in any order in ROS 2?** Discovery is dynamic and
  latched (TRANSIENT_LOCAL) topics deliver the last sample to late subscribers, so
  set-once data (map, robot_description) survives ordering; design callbacks to guard
  against not-yet-arrived inputs.
- **Q: What does `static_transform_publisher` do vs a dynamic TF broadcaster?**
  Publishes a fixed parent→child transform once/latched; used for rigid, unchanging
  relationships (sensor mounts, or here a known offset). Dynamic broadcasters
  republish a changing transform every cycle.
