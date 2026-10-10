# 08 — QoS, Frames & Callbacks (the A\* planner node, explained)

Reference notes for the concepts behind `vto_planning/astar_planner_node.py`. These
are three things every ROS 2 robotics interview probes, and all three show up in
this one node.

---

## 1. QoS: Reliability vs. Durability

A **QoS profile** is a contract between a publisher and a subscriber about *how*
messages are delivered. Two independent axes matter most:

### Reliability — "will each message arrive at all, while connected?"
- `RELIABLE` — retransmit until acknowledged; no loss (TCP-like).
- `BEST_EFFORT` — send once, may drop (UDP-like).
- Use RELIABLE for commands, maps, plans. Use BEST_EFFORT for high-rate sensor
  streams (lidar/camera) where the *latest* sample matters more than completeness.

### Durability — "will a subscriber that joins LATE get messages sent BEFORE it joined?"
- `VOLATILE` — no; you only receive messages published after you subscribe.
- `TRANSIENT_LOCAL` — the publisher keeps the last N samples and delivers them to
  late joiners. This is ROS 2's "latching."
- Use TRANSIENT_LOCAL for **set-once** data: the map, `robot_description`, a one-shot plan.

### They combine independently
The map is `RELIABLE` + `TRANSIENT_LOCAL` because `map_server` publishes the map
**once and stops**: a planner that starts *later* must still receive it
(transient-local), and must not drop it (reliable).

### Compatibility rule (this bites people)
The **publisher must offer at least what the subscriber requests.**
- A `TRANSIENT_LOCAL` subscriber gets **nothing** from a `VOLATILE` publisher.
- A `RELIABLE` subscriber won't connect to a `BEST_EFFORT` publisher.

That's why the `/map` subscription must *match* map_server's latched profile — a
mismatch means the node silently never receives a map.

---

## 2. The three origins (do NOT conflate these)

The planner juggles three different "(0,0)"s. Mixing them up is the #1 source of
"my path is offset by 10 metres" bugs.

| Origin | Value (in world) | What it is | Stored in the node as |
|--------|------------------|-----------|-----------------------|
| **World / `map` frame** | (0, 0) | Gazebo world centre — the global reference | — (the reference) |
| **Occupancy-grid corner** | **(−10, −15)** | Bottom-left of the grid; cell (0,0) lives here | `map_ox`, `map_oy` (from map YAML `origin:`) |
| **`odom` frame** | (−9.10, −14.09) | Robot's spawn pose | `spawn_x`, `spawn_y`, `spawn_yaw` |

- `map_ox/map_oy` drive **world ↔ cell**: `col = (x − map_ox)/res`, `row = (y − map_oy)/res`.
  They are NOT (0,0) — cell (0,0) is the grid's *corner*, which sits at world (−10, −15).
- `spawn_*` drive **world ↔ odom**: the robot's odom frame starts at its spawn pose.

### Why we hand-code the transform instead of using tf2
The follower (`pure_pursuit`) reads `/plan` directly against `/odom` with **no tf**,
so `/plan` must be in the **`odom` frame**. But the map is in the **world** frame.
The link between them, `map → odom`, is a rigid transform equal to the **spawn pose**.

In this node that transform is hand-coded in `_world_from_odom` / `_odom_from_world`
— there is **no tf2 lookup**. Why? Because `map → odom` is normally the *output of
localization* (AMCL/EKF), and we don't run localization yet (that's Phase 2). Since
we place the robot ourselves, we know the spawn exactly and substitute a constant —
"poor-man's localization."

The "proper" version replaces those two helpers with:
```python
tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
```
and lets localization keep `map → odom` continuously corrected for drift. **That one
swap is the entire difference between a static offset and real localization.**

Data flow each plan:  `odom pose → world → grid cell → A* → grid cell → world → odom → /plan`.

---

## 3. Callbacks: when does each fire?

Callbacks are **event-driven** — you never call them. The **executor** (running
inside `rclpy.spin(node)`) invokes a callback when a message lands on its topic.

| Callback | Fires when… | Frequency | Role |
|----------|-------------|-----------|------|
| `_on_map` | a msg arrives on `/map` | **~once** at startup (map_server latches, publishes once) | build grid + precompute clearance field |
| `_on_odom` | a msg arrives on `/odom` | **continuous**, tens of Hz (diff_drive_controller streams it) | cache latest `self.robot_odom` |
| `_on_goal` | a msg arrives on `/goal_pose` | **on demand**, per RViz "2D Nav Goal" click | run A*, publish `/plan` |

### Single-threaded executor = no races
`rclpy.spin` uses a single-threaded executor by default, so these callbacks run
**one at a time, never overlapping**. That's *why* it's safe for `_on_goal` to read
`self.robot_odom` while `_on_odom` writes it — there's no concurrent access. Two
callbacks just cache state (`_on_map`, `_on_odom`); only `_on_goal` acts.

---

## Interview angle

- **Q: Reliability vs. durability?** Reliability = per-message delivery guarantee
  *while connected* (RELIABLE vs BEST_EFFORT). Durability = whether *late* joiners get
  *past* messages (TRANSIENT_LOCAL vs VOLATILE). Independent axes.
- **Q: Why is a map published TRANSIENT_LOCAL?** It's set-once; consumers that start
  after the publisher must still receive the last value ("latching").
- **Q: A subscriber isn't receiving anything — QoS causes?** Incompatible profiles:
  VOLATILE publisher vs TRANSIENT_LOCAL subscriber, or BEST_EFFORT pub vs RELIABLE sub.
  Publisher must offer ≥ what the subscriber requests.
- **Q: Difference between the `map` and `odom` frames?** `map` is a fixed global
  frame (no drift, can jump when localization corrects). `odom` is smooth and
  continuous but drifts over time. `map → odom` is published by localization and
  absorbs the drift; `odom → base_link` comes from wheel odometry.
- **Q: What does AMCL/EKF actually give you?** The `map → odom` transform — exactly the
  constant we hard-code here as the spawn offset.
- **Q: How does a ROS 2 node decide when to run a callback?** The executor dispatches
  callbacks as messages arrive; a single-threaded executor serialises them, so shared
  node state needs no locking.
- **Q: Why cache odom in a member instead of planning inside `_on_odom`?** Planning is
  expensive and only needed on a new goal; `_on_odom` is high-rate, so it just updates
  state and returns. Separation of "sense" (cache) from "act" (plan on goal).
