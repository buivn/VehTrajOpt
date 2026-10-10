# 05 — Pure Pursuit: The Geometric Path Follower

Phase 1 begins. Phase 0 made the robot *drivable* (a `Twist` moves it); now we give
it a **path** and a **controller** that turns that path into velocity commands.
Pure Pursuit is the simplest such controller — pure geometry, no dynamics — so it's
the ideal baseline before MPC and MPPI.

Files:
[`pure_pursuit_node.py`](../ros2_ws/src/vto_control/vto_control/pure_pursuit_node.py),
[`path_publisher_node.py`](../ros2_ws/src/vto_control/vto_control/path_publisher_node.py),
[`pursuit_sim.launch.py`](../ros2_ws/src/vto_bringup/launch/pursuit_sim.launch.py).

Prereq: [doc 02](02-ros2-control-diff-drive.md) (kinematics, `/cmd_vel`, `/odom`).

---

## 1. The idea in one sentence

> Look at a point on the path a fixed distance ahead ("the carrot"), and steer
> along the circular arc that connects where you are to that point.

Like a human driver fixing their eyes on a spot down the road and steering toward
it. Move forward, the carrot moves forward, you keep curving toward it — and the
path gets followed.

```
                 * lookahead point (carrot), Ld away on the path
                /:
               / :
    arc the   /  : y_r   the robot follows this arc,
    robot    /   :        recomputed every cycle
    drives  /    :
   ┌────┐  /     :
   │ 🤖 │ · · · ·      (robot frame: x_r forward, y_r left)
   └────┘   x_r
```

---

## 2. The one equation

Put the robot at the origin of its own frame: **x** forward, **y** left. Let the
carrot be at `(x_r, y_r)` in that frame, a distance `Ld` (the *lookahead*) away.

There's exactly one circular arc that starts at the robot (tangent to its heading)
and passes through the carrot. Its **curvature** (1/radius) is:

```
        2 · y_r
  κ  =  ─────────
          Ld²
```

That's the whole controller. Intuition:
- carrot **dead ahead** (`y_r = 0`) → `κ = 0` → drive **straight**.
- carrot **to the left** (`y_r > 0`) → `κ > 0` → **curve left**.
- carrot **far away** (large `Ld`) → smaller `κ` → **gentler** correction.

Then convert curvature to a velocity command using the diff-drive relation from
doc 02 (`ω = v · κ`):

```
  v = cruise speed         (a chosen forward speed)
  ω = v · κ = v · 2·y_r / Ld²
```

Publish `(v, ω)` as a `TwistStamped` on `/cmd_vel`, and `diff_drive_controller`
turns it into wheel speeds. Done.

> **Where the equation comes from:** a chord of length `Ld` subtending a circle of
> radius `R` sits at lateral offset `y_r = Ld²/(2R)` from the tangent. Solve for
> `1/R`: `κ = 2·y_r/Ld²`. Worth being able to sketch in an interview.

---

## 3. Our implementation, walked through

[`pure_pursuit_node.py`](../ros2_ws/src/vto_control/vto_control/pure_pursuit_node.py):

1. **Inputs** — subscribe `/plan` (`nav_msgs/Path`) and `/odom`
   (`nav_msgs/Odometry`). Output `/cmd_vel` (`TwistStamped`). Same *interface* MPC
   and MPPI will use → swappable.

2. **Every odom message** (`_on_odom`), run one control step:
   - Get the robot pose `(px, py, yaw)`. Yaw comes from the quaternion via
     `atan2(2(wz+xy), 1−2(y²+z²))` — the quaternion→yaw formula (doc 00a: ground
     robots only use yaw).
   - **Find the lookahead point** (`_lookahead_point`): first find the path point
     *closest* to the robot (`_closest_index`, tracks progress), then search
     **forward** from there for the first point ≥ `Ld` away.

     > **Real bug we hit:** the first version scanned the path from the very
     > beginning. Once the robot moved a little, the first point ≥ `Ld` away was a
     > point *behind* it (back near the path start) → the carrot flipped behind →
     > the robot alternated "drive / spin in place / drive / spin" and got stuck
     > 0.4 m in. Searching *forward from the closest point* fixes it: the carrot is
     > always ahead on the path. This progress-tracking is essential, not optional.
   - **Transform the carrot into the robot frame** — rotate the world offset
     `(dx, dy)` by `−yaw`:
     ```
     x_r =  cos(−yaw)·dx − sin(−yaw)·dy
     y_r =  sin(−yaw)·dx + cos(−yaw)·dy
     ```
     This is exactly an SE(2) transform (doc 00a) — putting the target into "what's
     ahead / what's to my left" terms.
   - **Curvature → command**: `κ = 2·y_r/dist²`, then `ω = v·κ`, clamped to
     `max_angular`. Publish.

3. **Stopping** — when within `goal_tolerance` of the *last* path point, publish
   zero velocity and latch `reached`. Without this the robot orbits the goal
   forever.

4. **Edge case** — carrot behind the robot (`x_r < 0`): set `v = 0` and turn in
   place, so it recovers instead of driving away.

### The test path

[`path_publisher_node.py`](../ros2_ws/src/vto_control/vto_control/path_publisher_node.py)
is a **stand-in planner** (the real C++ planner arrives in Phase 1.5). It emits a
smooth quarter-circle arc from `(0,0)` curving left to `(2,2)` in the `odom` frame,
re-published on a 1 s timer so late subscribers still get it. The Phase 1 test
asserts the robot ends within 0.5 m of `(2,2)`.

---

## 4. The one knob that matters: lookahead `Ld`

Pure Pursuit lives or dies by its lookahead distance:

| `Ld` too small | `Ld` too large |
|---|---|
| Tight tracking | Smooth motion |
| **Oscillates / weaves** | **Cuts corners**, wide turns |
| Reacts to every wiggle | Sluggish to react |

Production systems use **adaptive lookahead** — `Ld` grows with speed (look farther
when moving fast). Nav2's *Regulated Pure Pursuit* controller does exactly this,
plus slows down on tight curvature. Our fixed `Ld = 0.6 m` is the teaching version.

---

## 5. What Pure Pursuit ignores (and why MPC/MPPI come next)

Pure Pursuit is a **kinematic, geometric** law. It does **not** know about:
- **Dynamics** — mass, momentum, wheel slip, actuator limits. It assumes you can
  instantly achieve any `(v, ω)`.
- **Constraints** — it won't respect "don't exceed 0.5 g lateral" or obstacle
  keep-outs.
- **Optimality / prediction** — it reacts to one carrot; it doesn't *plan ahead*
  over a horizon.

That's precisely the gap **MPC** (optimize a dynamics-aware trajectory over a
horizon subject to constraints) and **MPPI** (sample many rollouts, GPU-friendly)
fill. Pure Pursuit is the baseline they must beat — and because all three share the
`Path + Odometry → /cmd_vel` interface, `vto_bench` can compare them head-to-head.

---

## Run it

Inside the container (`source install/setup.bash` first):

```bash
# watch it follow the arc in Gazebo + rviz
ros2 launch vto_bringup pursuit_sim.launch.py

# automated check: follows the path, asserts it reaches ~(2,2)
bash scripts/phase1_pursuit_test.sh          # exit 0 = PASS

# inspect while it runs (new shell into the container)
ros2 topic echo /cmd_vel                       # the velocity commands
ros2 topic echo /plan --once                   # the path being followed
ros2 topic echo /odom --field pose.pose.position
```

Full command list in the [commands cheatsheet](commands.md).

---

## Interview angle

**Q: Explain Pure Pursuit.**
A geometric path tracker: pick a lookahead point on the path a fixed distance
ahead, compute the curvature of the arc from the robot to it (`κ = 2·y_r/Ld²` in
the robot frame), and command `ω = v·κ`. Recompute each cycle.

**Q: What does the lookahead distance trade off?**
Small `Ld` → accurate but oscillatory; large `Ld` → smooth but cuts corners.
Adaptive lookahead (scale with speed) balances them.

**Q: Derive the curvature formula.**
A chord of length `Ld` on a circle of radius `R` has lateral offset
`y_r = Ld²/(2R)`; solve for curvature `κ = 1/R = 2·y_r/Ld²`.

**Q: Limitations vs MPC/MPPI?**
No dynamics, no constraints, no prediction/optimization — it's a reactive kinematic
law. MPC/MPPI optimize over a horizon with a dynamics model and constraints.

**Q: Why transform the target into the robot frame?**
Curvature is naturally expressed relative to the robot's heading (`x_r` forward,
`y_r` left). The transform is an SE(2) rotation by `−yaw`.

**Q: Why search for the lookahead point forward from the closest path point?**
So the carrot is always *ahead* on the path. If you scan from the path start, once
the robot passes a point the "first point ≥ Ld away" can be one *behind* it,
flipping the carrot backward and making the robot oscillate. Track progress
(closest index) and search forward.

---

## Where this maps in our repo

- Controller: [`pure_pursuit_node.py`](../ros2_ws/src/vto_control/vto_control/pure_pursuit_node.py)
- Test path: [`path_publisher_node.py`](../ros2_ws/src/vto_control/vto_control/path_publisher_node.py)
- Launch: [`pursuit_sim.launch.py`](../ros2_ws/src/vto_bringup/launch/pursuit_sim.launch.py)
  (`ros2 launch vto_bringup pursuit_sim.launch.py`)
- Test: `ros2_ws/scripts/phase1_pursuit_test.sh` (asserts goal reached)
- **Next:** wrap the C++ core planner as the `/plan` source (Phase 1.5), then MPC.
