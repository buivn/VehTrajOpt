# 00 — The ROS 2 Robot Stack: The Big Picture

Before any code, build the mental model. Every robot we make in this repo —
mobile, car-like, snake — is the *same skeleton* with different parts swapped in.
If you understand this skeleton, the rest is detail.

---

## 1. What is ROS 2, really?

ROS 2 is **not an operating system** and **not a robot brain**. It is a
*middleware* — a standard way for many small programs to:

1. **run independently** (each is a **node**), and
2. **talk to each other** over a network (via **topics**, **services**, **actions**).

Think of it as "microservices for robots." Instead of one giant program, a robot
is ~20 little programs, each doing one job (read the LiDAR, estimate position,
plan a path, spin the wheels), all publishing and subscribing to messages.

**Why this design?** Because robotics is modular and heterogeneous. You want to
swap the LiDAR driver without touching the planner, run the camera node on a GPU
box and the motor node on a microcontroller, and reuse a navigation stack across
totally different robots. Loose coupling makes that possible.

> **Under the hood:** ROS 2 talks over **DDS** (Data Distribution Service), a
> real-time pub/sub standard. That's the big change from ROS 1 — no central
> `roscore` master; nodes discover each other peer-to-peer. Good interview point.

---

## 2. The five building blocks

| Concept | One-line definition | Real example in our repo |
|---|---|---|
| **Node** | One process doing one job | `robot_state_publisher`, `diff_drive_controller`, our `pure_pursuit` |
| **Topic** | A named, typed, many-to-many message stream (pub/sub, async) | `/cmd_vel`, `/odom`, `/scan` |
| **Message** | The typed data on a topic | `geometry_msgs/Twist` (velocity), `nav_msgs/Odometry` |
| **Service** | Request/response, synchronous (like a function call) | "reset odometry", "load a controller" |
| **Action** | Long-running goal with feedback + cancel | "navigate to (x,y)" — takes seconds, streams progress |

Rule of thumb: **streaming data → topic**, **quick command → service**,
**long task with progress → action**.

```
            /scan (LaserScan)          /cmd_vel (Twist)
 [lidar node] ───────────► [planner node] ───────────► [motor node]
              topic                        topic
```

---

## 3. TF: the thing that makes robotics *robotics*

The single most important robot-specific concept: **TF2**, the transform system.

A robot is a bunch of coordinate frames — the world, the robot base, each wheel,
the LiDAR, the camera. TF2 continuously tracks **where every frame is relative to
every other frame, over time**. It answers: *"this obstacle is 3 m in front of
the LiDAR — where is that in the map?"*

The standard frame chain (memorize this — it's a classic interview question):

```
map  ──►  odom  ──►  base_link  ──►  wheels / sensors
 │          │            │
 │          │            └─ rigid, from URDF (fixed offsets)
 │          └─ continuous but drifting (from wheel odometry)
 └─ discrete corrections, no drift (from localization / SLAM)
```

- **`map → odom`**: published by localization (AMCL, SLAM, robot_localization).
  Jumps occasionally to correct drift. Drift-free long-term.
- **`odom → base_link`**: published by the odometry source (wheel encoders).
  Smooth and continuous, but drifts over time.
- **`base_link → everything on the robot`**: fixed, comes straight from the URDF.

The genius: your **odom** transform can be smooth (good for control), while
**map** stays accurate (good for planning) — and TF composes them for you.

---

## 4. URDF: the robot's body

**URDF** (Unified Robot Description Format) is an XML file describing the robot's
physical structure: **links** (rigid bodies) connected by **joints**.

- **Link** = a rigid part. Has 3 properties: `visual` (what you see),
  `collision` (simplified shape for physics), `inertial` (mass + inertia tensor).
- **Joint** = a connection between two links. Types: `fixed`, `continuous`
  (wheels), `revolute` (arm, with limits), `prismatic` (slider).

We write URDF using **xacro** (XML macros) so we don't repeat ourselves — a
`wheel` macro instead of copy-pasting the left and right wheel. That's exactly
what `diffbot.urdf.xacro` does.

From the URDF, `robot_state_publisher` computes the fixed part of the TF tree.

> **Non-holonomic connection:** the URDF describes *geometry*, but your robot's
> *motion constraints* (a car can't move sideways) live in the controller and
> the dynamics model — not the URDF. Keep those two ideas separate.

---

## 5. ros2_control: the muscle layer

Between "I want to move at 0.5 m/s" and "spin this motor at 12 rad/s" sits
**ros2_control**. It's a framework with two halves:

- **Hardware interface** — talks to actual motors (or, in sim, to Gazebo). Exposes
  `command_interfaces` (what you can set: velocity, position) and
  `state_interfaces` (what you can read: encoder position/velocity).
- **Controllers** — plugins that convert high-level commands into joint commands.
  E.g. `diff_drive_controller` turns a `Twist` (linear+angular velocity) into
  left/right wheel velocities using the wheel-separation geometry.

The beauty: the **same controller** runs in sim and on real hardware — you only
swap the hardware interface underneath. That's why we chose it.

```
/cmd_vel (Twist)
     │
     ▼
diff_drive_controller ──► left/right wheel velocity commands
     │                          │
     └──────────────────────────┴──► hardware interface (Gazebo, or real motors)
                                          │
     odometry ◄────────────────────────  reads back wheel encoders
```

---

## 6. Simulation: Gazebo (and why)

You don't test control laws on a real robot first — you'd break it. **Gazebo
Harmonic** is a physics simulator that:

- loads your URDF as a physical body,
- simulates gravity, friction, collisions, sensors (LiDAR, camera, IMU),
- runs the *same* ros2_control hardware interface, so your controllers don't
  know they're in a simulation.

**`ros_gz_bridge`** connects Gazebo's internal messages to ROS 2 topics (e.g.
the `/clock` so everything uses simulation time via `use_sim_time`).

Later, **MuJoCo** enters for *learning* (RL) because it simulates contact
dynamics faster and is friendlier to massively-parallel training. Gazebo is for
integration/testing; MuJoCo is for training policies.

---

## 7. The whole pipeline: sense → think → act

Here is the full loop every autonomous robot runs. Our phases build it piece by
piece, bottom to top:

```
        ┌─────────────────────────────────────────────────────┐
        │                    PERCEPTION                        │
 SENSE  │  LiDAR / camera / IMU / wheel encoders / GPS         │  Phase 2
        └───────────────────────┬─────────────────────────────┘
                                │ raw sensor topics
        ┌───────────────────────▼─────────────────────────────┐
        │              STATE ESTIMATION / FUSION               │
        │   robot_localization EKF, Kalman filter → "where am I?"│ Phase 2
        └───────────────────────┬─────────────────────────────┘
                                │ /odom, TF map→odom→base_link
        ┌───────────────────────▼─────────────────────────────┐
 THINK  │                  PLANNING (Nav2)                     │
        │   global path + local trajectory (your C++ planner)  │ Phase 1.5 / 2
        └───────────────────────┬─────────────────────────────┘
                                │ nav_msgs/Path
        ┌───────────────────────▼─────────────────────────────┐
        │            CONTROL (path → velocity)                 │
        │   Pure Pursuit / MPC / MPPI  → geometry_msgs/Twist   │ Phase 1
        └───────────────────────┬─────────────────────────────┘
                                │ /cmd_vel
        ┌───────────────────────▼─────────────────────────────┐
 ACT    │         ros2_control → motors → wheels               │ Phase 0
        └─────────────────────────────────────────────────────┘
```

Notice we're building **bottom-up**: Phase 0 gets *act* working (a robot that
moves), Phase 1 adds *control*, Phase 2 adds *sense* + *think*. You always want a
moving robot first, then smarter and smarter commands feeding it.

---

## Interview angle

**Q: What's the difference between a topic, a service, and an action?**
Topic = async pub/sub stream (sensor data, commands). Service = sync
request/response (quick query). Action = long-running goal with feedback and
cancellation (navigation). Pick by duration + whether you need a reply.

**Q: Explain the map → odom → base_link transform chain.**
`base_link` is the robot body. `odom → base_link` comes from odometry: smooth and
continuous but drifts. `map → odom` comes from localization: it makes discrete
corrections so the robot is globally accurate. Control likes the smooth odom
frame; planning likes the accurate map frame; TF composes them.

**Q: Why ros2_control instead of publishing motor commands directly?**
Hardware abstraction. The same controller (e.g. diff_drive) runs unchanged in
simulation and on real hardware — you only swap the hardware interface. Plus you
get standard controller lifecycle, safety limits, and reuse.

**Q: What changed from ROS 1 to ROS 2?**
No central master — peer-to-peer discovery via DDS; real-time friendly; better
multi-robot and security support; lifecycle-managed nodes.

**Q: What does URDF describe, and what does it *not*?**
It describes the robot's rigid-body structure (links, joints, mass, geometry). It
does **not** describe motion constraints (non-holonomy), control laws, or
sensor-processing — those live in controllers, planners, and dynamics models.

---

## Where this maps in our repo

- Nodes/topics/TF/URDF/ros2_control/Gazebo — Phase 0, already scaffolded in
  `ros2_ws/` (`vto_description`, `vto_simulation`, `vto_bringup`).
- Next doc (01) dissects `vto_description` line by line — the robot's body.
