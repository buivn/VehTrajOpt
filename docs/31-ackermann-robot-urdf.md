# 31 — Ackermann robot: body/URDF (step 1 of the build)

Step 1 of 4 in building the Ackermann-steering robot: **just the body** —
links, joints, geometry — no `ros2_control` interfaces, no Gazebo plugins
yet. Mirrors how diffbot itself was built (docs/01 body → docs/02
ros2_control → docs/03 Gazebo): the kinematic skeleton has to exist and be
sane before wiring actuation onto it.

## 1. Theory: what changes vs. diff-drive, mechanically

A diff-drive robot's two wheels are both **actuated** and independently
commandable — (v, ω) maps directly to (left wheel speed, right wheel
speed), and ω can be nonzero at v=0 (turn in place). An Ackermann (car-like)
robot has exactly **2 actuated degrees of freedom** regardless of wheel
count — drive speed and a single effective steering angle `δ` — and every
other wheel motion is a *kinematic consequence* of those two, via the
bicycle model:

```
ẋ = v·cos(θ),  ẏ = v·sin(θ),  θ̇ = v·tan(δ) / L
```

where `L` is the **wheelbase** (front-to-rear axle distance). This is the
same equation `MPStandardSimulator`/`MPSnakeSimulator` in `src/GP/` already
use (`MPSnakeSimulator.cpp:167`: `ds[STATE_THETA] = v*tan(steer)/m_bodyLength`)
— confirming the bicycle model is the right abstraction to build the URDF
around, not a from-scratch derivation.

Two mechanical consequences that don't exist for diff-drive:
- **Minimum turning radius** `R_min = L / tan(δ_max)` — the robot physically
  cannot pivot in place. This is why A* (pivot-heavy paths) doesn't fit this
  robot and the Ackermann lattice (docs 24-25, Dubins-primitive-based)
  exists.
- **A steering joint is a genuinely separate joint from the wheel's spin
  joint** — a front wheel needs to both *turn left/right* (steering) and
  *spin* (rolling), which are two independent rotations about two different
  axes, so each front wheel needs a 2-joint chain, not diffbot's 1 joint per
  wheel.

## 2. Decisions (made with the user this step)

- **Size match = chassis footprint.** `base_len/base_wid/base_hei/base_mass`
  copied verbatim from `diffbot.urdf.xacro:8-11` (0.40 × 0.30 × 0.12 m,
  5.0 kg) — this is what the planners' `robot_radius`/inflation math cares
  about, so A*/lattice comparisons across the two robots stay apples-to-apples.
  Wheelbase becomes a free parameter *inside* that fixed box.
- **4-wheel, shared front steering angle** — 2 front wheels steer, 2 rear
  wheels are fixed-spin-only. Both front wheels get independent steering
  joints at the URDF level (not a `mimic` tag forcing them identical) —
  whether the eventual `ackermann_steering_controller` commands them
  identically or with true per-wheel Ackermann-angle geometry is a
  controller-config decision for step 3, not something to hardcode into the
  joint kinematics now.

## 3. Coding — design

One new file: `ackermannbot.urdf.xacro`, same directory as diffbot
(`ros2_ws/src/vto_description/urdf/mobile/`). Not a new package — this
robot shares `vto_description`'s existing macros
(`box_inertia`/`cyl_inertia`) and will share `vto_bringup`'s launch
structure once wired up (step 4).

**Two wheel macros, not one** (diffbot only needed one):
- `wheel` (rear) — identical shape to diffbot's existing macro: one
  `continuous` joint, `base_link` → wheel directly.
- `steer_wheel` (front) — a **2-joint, 3-link chain**:
  `base_link` --[revolute steer joint, axis z, limited ±`max_steer_angle`]-->
  `*_steer_link` (a small, near-massless "steering knuckle")
  --[continuous spin joint, axis y]--> `*_wheel` link.
  The knuckle link is required because a URDF joint connects exactly two
  links — steering and spinning are kinematically serial (the wheel's spin
  axis rotates *with* the current steer angle), so they can't be collapsed
  into one joint the way diffbot's single rigid spin axis could.

**Placement — reusing diffbot's own placement formula, not inventing a new
one.** Diffbot placed its casters at `x = ±(base_len/2 - wheel_radius)`
(`diffbot.urdf.xacro:100-101`) — inset from the body edge by exactly one
wheel radius so the wheel doesn't visually poke through the chassis box.
Same formula here, applied to axle placement:

| quantity | formula | value |
|---|---|---|
| wheelbase `L` | `base_len - 2*wheel_radius` | 0.40 − 0.16 = **0.24 m** |
| front/rear axle x | `±(base_len/2 - wheel_radius)` | ±0.12 m |
| track width | same value as diffbot's `wheel_separation` (same stance) | 0.34 m |
| wheel y | `±track/2` | ±0.17 m |

Steering limit: checked `src/GP/GPConstants.hpp:45-48` for the advisor's
default (`ROBOT_MIN/MAX_STEER_ANGLE = ±88°`, `ROBOT_BODY_LENGTH = 1.75`) to
see if there was a value worth reusing for continuity — there isn't:
those are unconstrained-testing defaults for a 1.75 m scenario, not a
realistic steering limit for a 0.40 m research robot, so reusing them
verbatim would be copying a number that doesn't mean what it looks like it
means. Picked **±0.5 rad (≈29°)** instead, a standard small-vehicle steering
range, and sanity-checked it against this project's own map:

```
R_min = L / tan(δ_max) = 0.24 / tan(0.5) ≈ 0.44 m
```

well under the maze's 1.5 m corridor width, so the chosen limit doesn't make
the robot undrivable in the existing environment.

No casters: diffbot's casters exist specifically to stabilize a **2-wheel**
base against pitching under acceleration (`diffbot.urdf.xacro:84-87`,
comment explains why). A 4-wheel base is inherently pitch-stable — casters
would be dead weight here, not a missing feature.

**Not yet in this file (deliberately deferred to later steps):** no
`xacro:include` of a `.ros2_control.xacro` or `.gazebo.xacro` — this file
alone won't spawn/drive in Gazebo yet. That's steps 2-3. Lidar/IMU links
also deferred to whichever step wires up Gazebo, since they're sensor
mounts, not body kinematics.

---

### _Interview angle_

**Q: Why can't you just give a car-like robot 4 independently-steerable
wheels and treat it like an omnidirectional platform?**
A: You physically *can* build such a vehicle (it'd be a different robot —
closer to a swerve-drive), but a standard Ackermann/car has a single
mechanical steering linkage front wheels are coupled to (or, per-wheel motors
with steer angles constrained to satisfy the Ackermann condition, avoiding
tire scrub) — it's a design constraint of the vehicle class being modeled,
not a limitation of URDF. Since we're modeling a car-like robot specifically
(to compare against the advisor's `MPStandardSimulator`, which assumes
exactly this bicycle model), the rear wheels are deliberately non-steered.

**Q: Why does the steering joint need its own link (the "knuckle") instead
of just adding a second axis to the wheel joint?**
A: A URDF joint has exactly one axis and one degree of freedom by
construction. Steering (rotation about the vertical axis) and rolling
(rotation about the wheel's own axle) are two independent, serially-composed
rotations — the wheel's rolling axis direction itself changes as the
steering angle changes. That's precisely what a 2-joint kinematic chain
represents and a single joint cannot.

**Q: Why is minimum turning radius `L / tan(δ_max)` and not just some
tuned constant?**
A: It falls directly out of the bicycle model's geometry: at max steering
angle, the front and rear axles' velocity vectors intersect at a point (the
instantaneous center of rotation) whose distance from the rear axle is
`L / tan(δ)` — smaller wheelbase or larger max steering angle both tighten
the turning circle. It's a derived consequence of `L` and `δ_max`, not a
free parameter to fit separately.

## Revision: chassis aspect ratio (length = 1.5 × width)

Feedback after the first build: matching diffbot's exact footprint
(0.40 × 0.30 m, ratio ≈ 1.33:1) doesn't read as very car-like — real cars
run notably longer relative to their width. Changed to a fixed **1.5:1
length:width ratio**, a standard car-like proportion, superseding the
original "match diffbot's footprint exactly" choice for *length*
specifically.

**What was held fixed vs. derived:** width kept at diffbot's 0.30 m,
length set to `1.5 * width = 0.45 m` — not the alternative of holding
footprint *area* constant and solving for both dimensions. Picked the
simpler option (change one dimension, not two) since the original
"match diffbot" rationale was about keeping planner/inflation comparisons
apples-to-apples via `robot_radius`, which this revision already loosens
slightly by design (a purpose-shaped car is not the same footprint as a
generic diff-drive robot) — there's no remaining reason to also preserve
area once exact-footprint-matching is off the table.

**What that changes downstream**, all consequences of the wheelbase/axle
formulas already being `base_len`-relative (`wheelbase = base_len -
2*wheel_radius`, `axle_x = base_len/2 - wheel_radius`) rather than
hardcoded — the geometry updated by editing exactly one property:

| quantity | before | after |
|---|---|---|
| chassis (L × W × H) | 0.40 × 0.30 × 0.12 m | 0.45 × 0.30 × 0.12 m |
| wheelbase | 0.24 m | 0.29 m |
| axle offset from center | ±0.12 m | ±0.145 m |
| min turning radius (`L/tan(0.5)`) | 0.44 m | 0.53 m |
| chassis mass | 5.0 kg | 5.625 kg |

Mass was **not** left at 5.0 kg: scaled with footprint area
(`5.0 * (0.45*0.30)/(0.40*0.30) = 5.625`) so density stays constant rather
than silently dropping as the box grows — resizing a physical body without
touching its mass is a common but real correctness gap (the inertia tensor
would still be internally consistent for the *new* box, but at the *old*
box's density, understating the chassis's actual rotational inertia).

**One YAML that doesn't auto-follow the URDF**: `wheelbase` is duplicated
by hand in `ackermannbot_controllers.yaml` (`ackermann_steering_controller`
has no way to read it from the URDF/xacro properties — controller params
and URDF geometry are two separate files with no shared source of truth
here) — updated 0.24 → 0.29 alongside. Worth remembering as a
standing gotcha for any future geometry change: two files to touch, not one.

**Re-verified**, not just re-parsed: `xacro`+`check_urdf` pass, box size
confirmed `0.45 × 0.30 × 0.12` in the rendered URDF, and the same
unicycle-equivalent drive test from docs/33 (`v=0.3, ω=0.4` for 4s) was
rerun — resulting odometry (`x=0.778, y=0.759`) matched the pre-revision
run almost exactly, as expected: the controller's inverse kinematics is
supposed to make the robot behave like the same commandable (v, ω) unicycle
regardless of wheelbase, only the internal steering angle differs to
compensate. That the trajectory came out unchanged is itself the
confirmation the wheelbase update didn't break anything.
