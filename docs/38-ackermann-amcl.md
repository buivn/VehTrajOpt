# 38 — Ackermann robot: AMCL (localization step 2 of 2)

Second and final localization piece. With EKF (docs/37) owning
`odom→base_footprint`, AMCL's job is the outer correction: `map→odom`,
using the lidar (docs/36) against the known map.

## 1. Theory (brief — full derivation in docs/11)

Monte Carlo Localization represents the robot's belief about its pose as a
weighted particle cloud, predict-updated each cycle: **predict** — move
every particle by the same odometry delta, with noise proportional to the
motion (a rotation injects more rotation-noise, a translation injects more
translation-noise — the four `alpha` parameters); **update** — reweight
particles by how well their predicted lidar scan matches the actual scan
against the known map (`likelihood_field` model — fast, smooth, scores
beam endpoints rather than full ray-casting); **resample** — draw a new
cloud favoring high-weight particles, count adapted via KLD-sampling.

## 2. Design decisions

**Reuse `nav2_amcl::DifferentialMotionModel`, don't invent an Ackermann
one.** `nav2_amcl` ships exactly two motion models: `Differential` and
`Omni`. Neither is a bicycle model, but that's less of a mismatch than it
sounds: `DifferentialMotionModel` operates on the *resulting* odometry pose
delta (decomposed as rotate → translate → rotate) between two filter
updates, not on commanded wheel velocities — it's agnostic to *how* that
delta was produced mechanically, only to the noise characteristics of
*getting* it. A car's odometry delta between two poses decomposes into the
same rot-trans-rot shape a diff-drive robot's does; what would differ is
the noise *magnitude* on each term (a car's dead-reckoning error grows
differently through a turn than a diff-drive robot's), which the `alphaN`
parameters already exist to tune per-robot, not per-model. This is the
same choice real Ackermann-vehicle AMCL setups make in practice — not a
compromise specific to this project.

**New config file** (`amcl_ackermann.yaml`), not a parametrized shared
one — same reasoning as docs/37: `set_initial_pose`'s `yaw` has to match
this robot's actual spawn heading (`1.5708`, the corridor-facing fix),
which diffbot's `amcl.yaml` hardcodes at `0.0` for its own (different)
spawn orientation. Getting this right matters more than it might look:
seeding the particle cloud at the wrong heading means AMCL's very first
correction has to fix a ~90° error immediately, defeating the entire point
of seeding a known start pose instead of requiring an RViz "2D Pose
Estimate" click.

**Extend `maze_ackermann.launch.py`, don't create a third file.** AMCL
needs `map_server` (serves the map) and a `lifecycle_manager` (activates
`map_server` then `amcl` in order) alongside it — these three are one
coherent localization stack, not independent pieces, so they belong in the
same file docs/37 already started, following the same "one file,
incrementally extended across steps" pattern as `ackermannbot.urdf.xacro`
(docs 31→36) and `ackermannbot.gazebo.xacro` (docs 33→36).

## 3. Coding

New file: `amcl_ackermann.yaml` — copy of `amcl.yaml` with `initial_pose.yaw:
1.5708` (was `0.0`) and `robot_model_type` explicitly left as
`nav2_amcl::DifferentialMotionModel` per the reasoning above (not a
silent omission — a future reader shouldn't have to re-derive why it's
not obviously wrong that a car-like robot uses a model literally named
"Differential"). Everything else (laser model, particle counts,
`update_min_d/a`) is map/sensor physics, not drivetrain-specific, copied
unchanged.

`maze_ackermann.launch.py`: three new nodes, same shape as
`maze_astar.launch.py`'s — `map_server` (same `maze.yaml`, shared map),
`lifecycle_manager_localization` (`node_names: ["map_server", "amcl"]`),
`amcl` (new config). No RViz added yet — this step is about correctness of
the localization stack itself, verified via topics/TF, matching how docs/37
was verified; a GUI pass is a separate, later "let's look at it" step, not
a correctness requirement.

## Verification plan

Spawn via the extended launch file, confirm: (1) `map_server` and `amcl`
both reach the `active` lifecycle state; (2) AMCL is the one now
broadcasting `map→odom` (not a static/absent transform); (3) `/amcl_pose`
publishes with tight covariance at the known-correct seeded pose (not
requiring a manual correction first); (4) after driving a short distance,
`map→odom` correction keeps the `map`-frame pose consistent with actual
motion (not diverging).

**Results:** all confirmed. `map_server` and `amcl` both reached `active`.
`/amcl_pose` was already at `(-9.1, -14.1)` with covariance ~1e-12
(essentially exact) immediately at startup — confirms the `initial_pose.yaw
= 1.5708` fix seeded the cloud correctly; a wrong yaw here would have shown
up as a large initial position/heading error instead. `ros2 topic info /tf
--verbose` listed `amcl` as a `/tf` publisher alongside `ekf_filter_node`;
`tf2_echo map odom` resolved translation `(-9.1, -14.1)`, yaw `1.571 rad`
(90°) — exactly the spawn pose, as expected at t≈0 before any drift.
Drove forward ~0.3 m/s briefly: `/amcl_pose` moved to `(-9.12, -13.28)` —
+0.82 m in the expected north (+y, this cell's only open corridor)
direction, ~0.02 m of lateral drift — consistent, non-diverging map-frame
tracking.

This closes out Ackermann localization (docs 36-38): sensors, EKF, and AMCL
all verified working together in the maze. What's still open before this
robot can actually navigate the maze (not just localize in it): a
kinematically-valid planner (the Ackermann lattice's ROS node, docs 24-25,
2/3 built) to feed `pure_pursuit_ackermann` (docs/35) a path.
