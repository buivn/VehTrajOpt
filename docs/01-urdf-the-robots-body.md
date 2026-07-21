# 01 — URDF: The Robot's Body

We now open the file we scaffolded in Phase 0 and read it top to bottom:
[`diffbot.urdf.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.urdf.xacro).
By the end you'll know what every tag does and *why* it's there.

Prereq: [doc 00 §4](00-ros2-robot-stack-overview.md) (what URDF is) and
[doc 00a](00a-tf-deep-dive.md) (frames/transforms).

---

## 0. What we're describing

A **differential-drive** robot: a box chassis, **two independently driven
wheels**, and one **passive caster** for balance. "Differential drive" =
steering by driving the two wheels at *different* speeds (both forward → straight;
left faster → turns right; opposite → spins in place). Simplest real robot;
perfect for validating the whole toolchain.

The URDF is a **tree of links connected by joints**. For our robot:

```
base_footprint
   └── base_link                (the chassis)
        ├── left_wheel          (continuous joint — spins)
        ├── right_wheel         (continuous joint — spins)
        └── caster              (fixed joint — just slides)
```

`robot_state_publisher` reads this tree and publishes the **fixed part of the TF
tree** from it (see doc 00a). So *this file literally becomes coordinate frames.*

---

## 1. The `<robot>` root and xacro (lines 1–5)

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="diffbot">
  <xacro:arg name="use_sim" default="true"/>
```

- **Plain URDF** is static XML — no variables, no math, no loops. Painful for
  anything with repeated parts.
- **xacro** ("XML macros") is a preprocessor that adds **properties** (constants),
  **macros** (functions), and **`${...}` math**. It runs *before* ROS sees the
  file and expands to plain URDF. `xacro diffbot.urdf.xacro` prints the final URDF.
- **`<xacro:arg>`** = a parameter you can pass in at launch
  (`xacro ... use_sim:=false`). Different from a property: args come from
  *outside*, properties are internal constants. We use `use_sim` to branch
  sim-vs-real later.

> **Interview distinction:** *URDF is the format; xacro is a templating layer that
> generates URDF.* The robot only ever "sees" the expanded URDF.

---

## 2. Properties: single source of truth (lines 7–17)

```xml
<xacro:property name="wheel_radius"     value="0.08"/>
<xacro:property name="wheel_separation" value="0.34"/>
```

These are **named constants** in SI units (meters, kilograms). Define a dimension
once, reference it everywhere with `${wheel_radius}`. Two payoffs:

1. Change the robot's size in *one* place.
2. The **same numbers** feed the controller config later — `wheel_radius` and
   `wheel_separation` appear again in
   [`diffbot_controllers.yaml`](../ros2_ws/src/vto_description/config/diffbot_controllers.yaml).
   The controller's kinematics *must* match the body's geometry, or odometry
   drifts. Keeping the numbers together (mentally) prevents that class of bug.

Everything is SI. ROS assumes **meters, kilograms, seconds, radians**. No cm, no
degrees. (`1.5708` on line 62 is π/2 radians = 90°.)

---

## 3. Inertia macros: giving parts *mass properties* (lines 19–36)

This is the part beginners skip and physics engines punish.

```xml
<xacro:macro name="box_inertia" params="m x y z">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${m*(y*y+z*z)/12.0}" ... />
  </inertial>
</xacro:macro>
```

A **macro** is a reusable function. `box_inertia` takes mass + dimensions and
emits an `<inertial>` block. `cyl_inertia` does the same for a cylinder (wheels).

### What is the inertia tensor and why compute it?

- **Mass** tells the physics engine how hard the part is to *push* (linear).
- The **inertia tensor** tells it how hard the part is to *rotate* (angular) — the
  rotational equivalent of mass. Without it (or with a wrong one), Gazebo produces
  garbage: robots that vibrate, sink, or launch into orbit.

The formulas are the **standard rigid-body inertia** of a uniform solid about its
center:

- Solid box: `I_xx = m(y² + z²)/12`, and cyclic permutations. (Lines 23–25.)
- Solid cylinder (axis along local Z): `I_zz = m·r²/2`,
  `I_xx = I_yy = m(3r² + l²)/12`. (Lines 32–34.)

We only fill the **diagonal** (`ixx, iyy, izz`) and set off-diagonals to 0 because
our shapes are symmetric about their axes — no products of inertia. Good enough
and standard for simple links.

> **Interview point:** "Why does a URDF link need `<inertial>`?" → Because the
> physics simulator integrates rigid-body dynamics; mass handles translation, the
> inertia tensor handles rotation. Missing/incorrect inertia is the #1 cause of
> unstable Gazebo robots.

---

## 4. Links: the `visual` / `collision` / `inertial` trio (lines 38–50)

```xml
<link name="base_link">
  <visual>    ...box, steel color... </visual>
  <collision> ...box...              </collision>
  <xacro:box_inertia .../>
</link>
```

Every physical link has **three independent descriptions of its shape**, and
knowing why they're separate is a classic question:

| Element | Answer to... | Fidelity |
|---|---|---|
| `<visual>` | "What does it look like?" | High — meshes, colors, textures |
| `<collision>` | "What does it bump into?" | **Low on purpose** — boxes/cylinders |
| `<inertial>` | "How does it move under force?" | Mass + inertia tensor |

**Why is collision deliberately simpler than visual?** Collision-checking runs
thousands of times per second. A detailed mesh (10k triangles) would kill
performance, so we approximate with primitives (box, cylinder, sphere). Here
visual and collision happen to be the same box, but on real robots the collision
shape is a coarse hull of a pretty mesh.

### `base_footprint` vs `base_link` (lines 39, 52–56)

```xml
<link name="base_footprint"/>            <!-- empty! no geometry -->
<joint name="base_joint" type="fixed">
  <origin xyz="0 0 ${wheel_radius + wheel_zoff}"/>
</joint>
```

- **`base_footprint`** is a *virtual* link with **no shape** — a point on the
  **ground directly under the robot** (z = 0). Convention: navigation and
  localization reason about the robot's footprint on the floor.
- **`base_link`** is the physical chassis, sitting at wheel-radius **height**. The
  fixed joint lifts it up by `wheel_radius + wheel_zoff` so the wheels touch z = 0.

This is a **standard ROS convention** (REP-105): `base_footprint` on the ground,
`base_link` at the body. Nav2 expects it.

---

## 5. Joints: how links connect and move (lines 52–56, 73–78)

A `<joint>` says "child link is attached to parent link, offset by `<origin>`,
and moves in this way."

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child  link="left_wheel"/>
  <origin xyz="0 ${side_y} ${-wheel_zoff}"/>
  <axis xyz="0 1 0"/>
</joint>
```

Key pieces:

- **`type`** — the motion allowed:
  - `fixed` — no motion (chassis→caster, footprint→base). Becomes a **static** TF.
  - `continuous` — unlimited rotation about one axis (**wheels**).
  - `revolute` — rotation *with* limits (an arm joint).
  - `prismatic` — sliding (a linear actuator).
- **`<origin>`** — where the child frame sits relative to the parent. `side_y`
  places left/right wheels ±half the separation apart. **This offset *is* the
  fixed transform** `T_base←wheel` that TF will publish.
- **`<axis>`** — for moving joints, the axis of motion. `0 1 0` = the wheel spins
  about the robot's **Y (side-to-side)** axis, i.e. rolls forward. Correct.

> The wheel *visual* is rotated `rpy="1.5708 0 0"` (line 62) because a URDF
> cylinder is born standing up (axis along Z); we tip it 90° so it lies like a
> wheel. That's cosmetic — the *joint* `<axis>` is what governs the spin.

---

## 6. The `wheel` macro: DRY in action (lines 59–82)

```xml
<xacro:macro name="wheel" params="prefix side_y"> ... </xacro:macro>

<xacro:wheel prefix="left"  side_y="${ wheel_separation/2}"/>
<xacro:wheel prefix="right" side_y="${-wheel_separation/2}"/>
```

Instead of copy-pasting two nearly identical wheel blocks, we write the wheel
*once* and **call it twice** with different `prefix` and `side_y`. `${prefix}`
builds unique names (`left_wheel`, `left_wheel_joint`). This is the entire point
of xacro — and it scales: the snake robot later will call a `segment` macro in a
loop for N identical body segments.

---

## 7. The caster (lines 84–94)

A **sphere on a fixed joint**. It doesn't steer or drive — it just slides to keep
the chassis level (the third contact point of a tripod). In the Gazebo file we
give it **zero friction** so it slides freely instead of fighting the drive
wheels. A real robot uses a ball-caster; a frictionless sphere is the standard
sim shortcut.

---

## 8. The includes: composition (lines 96–98)

```xml
<xacro:include filename="$(find vto_description)/urdf/mobile/diffbot.ros2_control.xacro"/>
<xacro:include filename="$(find vto_description)/urdf/mobile/diffbot.gazebo.xacro"/>
```

We split the robot across three files by **concern**:

- `diffbot.urdf.xacro` — the **body** (this file: links, joints, mass).
- `diffbot.ros2_control.xacro` — the **control interface** (doc 02).
- `diffbot.gazebo.xacro` — the **sim plugins & friction** (doc 03).

`$(find vto_description)` resolves to the installed package path. Splitting like
this means the *same body* can be reused with a different control or sim layer —
exactly the modularity ROS is built for.

---

## How to see the result (once built in Docker)

```bash
# expand xacro → plain URDF and eyeball it
xacro diffbot.urdf.xacro

# check it's a valid tree
check_urdf <(xacro diffbot.urdf.xacro)

# visualize the frames live
ros2 launch vto_description description.launch.py
# then in rviz: add RobotModel + TF displays
```

`check_urdf` prints the parent/child tree — a great sanity check that
`base_footprint → base_link → {wheels, caster}` is wired correctly.

---

## Interview angle

**Q: What are the three parts of a URDF link and why separate them?**
`visual` (appearance, high-detail meshes), `collision` (simplified shapes for fast
physics checks), `inertial` (mass + inertia tensor for dynamics). Separated
because rendering, collision-checking, and dynamics have different fidelity needs
— collision is kept coarse for speed.

**Q: Why does a link need an inertia tensor?**
The physics engine integrates rigid-body motion. Mass governs linear acceleration;
the inertia tensor governs angular acceleration. Wrong/missing inertia is the top
cause of unstable simulated robots.

**Q: URDF vs xacro?**
URDF is the XML robot format; xacro is a macro/templating preprocessor that
expands to URDF, adding constants, math, and reusable macros (DRY).

**Q: What's `base_footprint` vs `base_link`?**
`base_footprint` is a virtual ground-projected frame (z=0) used by nav/localization;
`base_link` is the physical body frame. Convention per REP-105.

**Q: Joint types?**
`fixed` (rigid, becomes static TF), `continuous` (unlimited rotation — wheels),
`revolute` (limited rotation — arm), `prismatic` (linear slide).

**Q: How does a URDF relate to TF?**
`robot_state_publisher` reads the URDF and publishes each joint's parent→child
`<origin>` as a transform — fixed joints go to `/tf_static`, movable joints get
updated on `/tf` from `/joint_states`.

---

## Where this maps in our repo

- File: [`diffbot.urdf.xacro`](../ros2_ws/src/vto_description/urdf/mobile/diffbot.urdf.xacro)
- Consumed by: `robot_state_publisher` in
  [`description.launch.py`](../ros2_ws/src/vto_description/launch/description.launch.py)
- **Next (doc 02):** `diffbot.ros2_control.xacro` +
  `diffbot_controllers.yaml` — how a `Twist` on `/cmd_vel` becomes wheel spin.
