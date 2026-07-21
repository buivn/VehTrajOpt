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
| — | [Commands cheatsheet](commands.md) | All build/run/drive/test/debug commands in one place |

## How we work

- One package / one function per step.
- I explain, then we build, then you read the doc.
- You set the pace — tell me "next" when ready, or ask to go deeper.
