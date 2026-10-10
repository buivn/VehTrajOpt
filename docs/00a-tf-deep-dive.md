# 00a — TF Deep Dive: Transforms, SE(n), and the `/tf` Topic

A Q&A companion to [doc 00](00-ros2-robot-stack-overview.md), section 3. This is
the single most-asked robotics interview topic, so it gets its own note.

---

## Q1. Is a TF transform just a matrix transformation?

Yes — each TF edge (e.g. `odom → base_link`) is a **rigid-body transform in
SE(3)**: a **rotation + a translation**, with *no* scaling or shearing. In
homogeneous form that's the 4×4 matrix:

```
        ┌               ┐
   T  = │   R (3×3)   t  │      R = rotation (3×3 orthonormal)
        │               │      t = translation (3×1)
        │  0  0  0    1  │
        └               ┘
```

The 4×4 (homogeneous) form exists so that **composition = matrix multiplication**
and translation rides *inside* the multiply.

### Storage vs mental model

TF does **not** put a raw 4×4 on the wire. It stores rotation as a **quaternion**
(4 numbers) + a **translation vector** (3 numbers). Why a quaternion instead of a
3×3 matrix or Euler angles?

- Compact (4 numbers, no 9-number redundancy).
- **No gimbal lock** (the singularity Euler angles suffer).
- Smooth to interpolate (SLERP).

> The 4×4 matrix is the *mental model*; quaternion + vector is the *storage
> format*. Libraries convert between them freely.

---

## Q2. SE(2) vs SE(3) vs "SE(4)"?

`SE(n)` = **Special Euclidean group** in `n`-dimensional space = all rigid-body
motions (rotation + translation) in `ℝⁿ`. **The number is the dimension of the
physical space, not the matrix size.**

| | Space | DOF | Homogeneous matrix | Used for |
|---|---|---|---|---|
| **SE(2)** | 2D plane | **3** — (x, y, θ) | **3×3** | Mobile robots on flat ground |
| **SE(3)** | 3D | **6** — (x, y, z, roll, pitch, yaw) | **4×4** | Drones, arms, full 3D |
| SE(4) | 4D | 10 | 5×5 | *Not used in robotics* |

**The trap:** a 4×4 matrix is **SE(3)**, *not* SE(4). The matrix is always
`(n+1)×(n+1)` — the extra row/column is the homogeneous trick.

- SE(2) → 3×3 (2×2 rotation block + 2×1 translation)
- SE(3) → 4×4 (3×3 rotation block + 3×1 translation)

### Which do we use?

Our diff-drive / car-like robots live on the ground → their pose is fundamentally
**SE(2)**: `(x, y, θ)`, 3 numbers. But **ROS/TF always stores SE(3)** (no 2D
mode). So a ground robot is an **SE(2) pose embedded in SE(3)**: `z = 0`,
`roll = pitch = 0`, and the quaternion encodes only yaw.

> Interview gold: say the robot is *planar (SE(2))* even though the *framework is
> SE(3)*. Shows you know the difference between the robot and the tooling.

---

## Q3. To get the robot's current pose: `pose = map→odom · odom→base`?

Yes. Writing `T_map←base` = "pose of `base_link` expressed in the `map` frame":

```
T_map←base  =  T_map←odom  ·  T_odom←base
```

The edges **chain**, and the shared middle frame (`odom`) cancels like units.
That product **is** the robot's pose in the map.

Two notes:

1. **You never write the multiply yourself.** You call the lookup and TF walks the
   tree:
   ```python
   tf_buffer.lookup_transform("map", "base_link", stamp)   # (target, source)
   ```
   It finds the path `map → odom → base_link` and multiplies internally.

2. **A transform is dual-purpose.** `T_map←base` is *both*:
   - "the pose of base_link in map", and
   - "the operator converting a point from base coords to map coords":
     `p_map = T_map←base · p_base`.

   Same matrix, two readings.

---

## Q4. Does `/tf` carry different edges with different `frame_id`s?

Yes. `/tf` is a **shared stream** many nodes publish to. Over a short window you
see interleaved messages:

```
# from the localization node:
frame_id: "map"    child_frame_id: "odom"       transform: {...}   stamp: t1

# from the wheel-odometry source:
frame_id: "odom"   child_frame_id: "base_link"  transform: {...}   stamp: t2
```

Each publisher owns its **own edge(s)** and publishes at its **own rate**
(localization ~10 Hz, odometry ~50 Hz → different timestamps). The **TF2 buffer**
on the subscriber collects them all into one coherent, timestamped tree.

Consequences:

- **One edge = one publisher.** Two nodes publishing the *same* edge (both
  claiming `odom→base_link`) is a classic bug — transforms fight, robot
  "teleports." **A frame has exactly one parent.**
- One `TFMessage` can bundle **several** `TransformStamped`s at once
  (`robot_state_publisher` emits all wheel/sensor edges together), so "one message
  = one edge" is not guaranteed — but "**one edge = one owner**" is.

### Message anatomy

`/tf` type is `tf2_msgs/TFMessage` = an array of `geometry_msgs/TransformStamped`:

```
header:
  stamp: 1721270400.123      # WHEN this transform is valid (crucial!)
  frame_id: "map"            # parent
child_frame_id: "odom"       # child
transform:
  translation: {x, y, z}     # the t vector
  rotation:    {x, y, z, w}  # the quaternion (R)
```

`/tf` = dynamic (changes over time). `/tf_static` = fixed edges
(`base_link→lidar`), published once and latched.

---

## Q5. Why is TF a *buffer*, not a lookup? (the time dimension)

Every transform is **timestamped**, and TF2 keeps a **rolling history** (a few
seconds). So you can ask:

> "Where was the LiDAR relative to the map **at the exact time this scan was
> taken**?"

TF2 **interpolates** between the two nearest buffered transforms to answer. That
time dimension is why TF is *spatio-temporal*, not just current-value.

---

## Interview one-liners

- **Transform:** "An SE(3) rigid-body pose — rotation stored as a quaternion plus
  translation. All transforms are timestamped `TransformStamped` messages on the
  shared `/tf` and `/tf_static` topics; TF2 buffers them into a time-indexed tree
  and composes chains by matrix multiplication."
- **SE(n):** "A ground robot's pose is SE(2) — (x, y, θ) — but ROS stores it in
  SE(3) with z, roll, pitch zeroed. SE(n) uses an (n+1)×(n+1) matrix, so a 4×4 is
  SE(3), not SE(4)."
- **Pose:** "`T_map←base = T_map←odom · T_odom←base`; `lookup_transform` computes
  it for you."
- **`/tf` ownership:** "Many publishers, one topic — but exactly one publisher per
  edge; a frame has one parent."

---

*Related: [doc 00](00-ros2-robot-stack-overview.md) §3–4. Next up: doc 01 — the
URDF walkthrough (the robot's body).*
