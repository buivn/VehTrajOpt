# VehTrajOpt — Learning Docs

These docs are written in **learning mode**: each one explains a concept or a
step we built, *why* it exists, and how it fits the larger robot pipeline. Read
them in order. Each ends with an **Interview angle** — questions you're likely
to get asked, with short answers.

## Index

| # | Doc | What it covers |
|---|-----|----------------|
| 00 | [ROS 2 robot stack overview](00-ros2-robot-stack-overview.md) | The big picture: nodes, topics, TF, URDF, control, sim — and the sense→plan→act pipeline |
| 00a | [TF deep dive](00a-tf-deep-dive.md) | Transforms as SE(n) matrices, quaternions, pose composition, the `/tf` topic, the time buffer |
| 01 | [URDF — the robot's body](01-urdf-the-robots-body.md) | Links, joints, visual/collision/inertial, inertia tensors, xacro macros, base_footprint |
| 02 | [ros2_control + diff drive](02-ros2-control-diff-drive.md) | Hardware interface vs controllers, diff-drive kinematics, odometry, the `/cmd_vel`→wheels flow |
| 03 | [Gazebo simulation + ros_gz bridge](03-gazebo-simulation.md) | SDF world, gz_ros2_control plugin, friction, the bridge, use_sim_time, launch ordering — **Phase 0 done** |
| 04 | [Environment setup: Docker + GPU + GUI](04-environment-setup.md) | Driver vs CUDA, snap vs Docker Engine, docker group/socket, nvidia-container-toolkit, X11 GUI |
| 05 | [Pure Pursuit](05-pure-pursuit.md) | Geometric path following: lookahead, curvature `κ=2y/Ld²`, TwistStamped `/cmd_vel`, limitations vs MPC/MPPI |
| 06 | [Maze environment](06-maze-environment.md) | Procedural 20×30 maze, exact 1.5 m corridors, recursive-backtracker, SDF generation |
| 07 | [A\* global planner (design)](07-astar-global-planner.md) | Planner vs controller, grid A\*, `f=g+h`, admissibility, clearance cost, the `vto_planning` design |
| 08 | [QoS, frames & callbacks](08-ros2-qos-frames-and-callbacks.md) | Reliability vs durability, the three origins (map/grid/odom), poor-man's localization, callback triggering |
| 09 | [Integration launch, lifecycle & map_server](09-integration-launch-lifecycle-and-map-server.md) | maze_astar.launch.py, lifecycle nodes + lifecycle_manager, static map→odom TF, why launch order is free |
| 10 | [Planning for the body & robust following](10-planning-for-the-body-and-robust-following.md) | C-space inflation, hard vs soft clearance, adaptive lookahead + turn-in-place + curvature slowdown, odom drift |
| 11 | [Localization with AMCL](11-localization-amcl.md) | map↔odom two-link chain, why sim odom drifts, particle filter (predict/update/resample), KLD-sampling, AMCL vs EKF |
| 12 | [Map-frame refactor](12-map-frame-refactor.md) | Why AMCL did nothing until the loop moved to the map frame; planner+follower read `map→base` via TF; frame gotchas |
| 13 | [MPC controller (design)](13-mpc-controller.md) | Receding horizon, unicycle model, N-step reference window, CasADi+IPOPT, tracking/effort/comfort costs, MPC vs Pure Pursuit vs MPPI |
| 14 | [Benchmarking controllers](14-benchmarking.md) | The 7 metrics, goal-tour vs teleport, the `vto_bench` harness, compute-time publishing, how to run + analyze |
| 15 | [MPPI controller (design)](15-mppi-controller.md) | Sampling-based MPC, path-integral weighting, temperature λ, MPPI vs MPC (sample vs gradient), GPU parallelism, nonconvex costs |
| 16 | [Nav pipeline diagram](16-nav-pipeline-diagram.md) | Static node/topic/TF reference for the full stack as wired: Gazebo → localization → planning → control, plus where `vto_bench` taps in |
| 17 | [Adding an IMU](17-imu-sensor.md) | Phase 2 step 1: simulated IMU on the diffbot publishing `/imu` — raw sensor only, no fusion yet |
| 18 | [robot_localization EKF](18-robot-localization-ekf.md) | Phase 2 step 2: `ekf_node` fuses `/odom` + `/imu` into `/odometry/filtered`, takes over `odom→base_footprint` TF |
| 19 | [EKF fusion mechanics](19-ekf-fusion-mechanics.md) | The internals of doc 18's `ekf_node`: predict/update tick-by-tick, no new files |
| 20 | [Trajectory generation roadmap](20-trajectory-generation-roadmap.md) | Scoping pass: method × robot-model matrix + fixed metrics list before writing any generation code |
| 21 | [DWA for diff-drive](21-dwa-diffdrive.md) | Dynamic Window Approach theory + design — **ruled out, not implemented** (§4 explains why); kept as the interview-worthy record of the trade-off |
| 22 | [State-lattice path generation, diff-drive](22-lattice-diffdrive.md) | First generation-method pipeline for M1 (diff-drive): theory + method selection + design, no code yet |
| 23 | [Multi-robot coordination roadmap](23-multi-robot-coordination-roadmap.md) | Orchestration-track scoping: task allocation, MAPF (prioritised → PBS → windowed), zone-based traffic, spatial decomposition, phased build order, fleet metrics |
| 24 | [Ackermann state lattice, step 1: Dubins](24-lattice-ackermann-dubins.md) | Bounded-curvature edge primitive for M2 (car-like): `dubins.py`, shortest drivable curve between two oriented poses |
| 25 | [Ackermann state lattice, step 2: control set + search](25-lattice-ackermann-control-set.md) | Turns Dubins edges into a minimal control set and runs the lattice search: `lattice_ackermann.py` |
| 26 | [Off-road planning method taxonomy](26-offroad-planning-method-taxonomy.md) | Reference doc: main off-road car-like planning methods, how they decompose, concrete disadvantages of each |
| — | [Trajectory Optimization report](report/trajectory-optimization-report.md) | Living comparison: planners (A\*, sampling) × controllers (Pure Pursuit, MPC, MPPI) — metrics + results |
| — | [Commands cheatsheet](commands.md) | All build/run/drive/test/debug commands in one place |

## How we work

- One package / one function per step.
- I explain, then we build, then you read the doc.
- You set the pace — tell me "next" when ready, or ask to go deeper.
