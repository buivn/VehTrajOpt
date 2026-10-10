# VehTrajOpt — ROS 2 workspace

ROS 2 Jazzy layer that wraps the existing C++ trajectory planner (in the repo
root `src/`, © Erion Plaku, used with permission) and drives simulated robots in
Gazebo Harmonic. Kept **separate** from the root plain-CMake project so `colcon`
and the legacy build never collide.

## Packages

| Package | Purpose | Phase |
|---|---|---|
| `vto_description` | URDF/xacro robot models (`mobile/` diffbot now; car-like, snake next) + `ros2_control` | 0 |
| `vto_simulation`  | Gazebo Harmonic world, spawn, `ros_gz` bridge | 0 |
| `vto_bringup`     | Top-level launch composing subsystems | 0 |
| `vto_control`     | Path trackers — Pure Pursuit (stub), MPC, MPPI | 1 |
| `vto_localization`| `robot_localization` EKF + sensor fusion | 2 (todo) |
| `vto_navigation`  | Nav2 params + custom controller plugins | 2 (todo) |
| `vto_bench`       | Eval / trajectory metrics / plots | cross-cutting |

## Build & run (inside the Docker container)

```bash
# --- on the host, once ---
xhost +local:root
cd docker && docker compose build && docker compose run --rm vto

# --- inside the container ---
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Phase 0: Gazebo + diffbot + controllers + rviz
ros2 launch vto_bringup sim.launch.py

# drive it (new shell, source install/setup.bash first)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

If the robot moves in Gazebo and its model + TF show in rviz, Phase 0 is done.

## Roadmap

- **Phase 0** — toolchain: Docker + diffbot spawns and drives. ← *you are here*
- **Phase 1** — control: Pure Pursuit → MPC (CasADi/acados) → MPPI (GPU). Common
  `Path → cmd_vel` interface; benchmark in `vto_bench`. Bridge the core C++
  planner here (`vto_planner_bridge`) to publish `nav_msgs/Path`.
- **Phase 2** — autonomy: add IMU/LiDAR/GPS to the URDF, `robot_localization`
  EKF fusion, then Nav2 with the Phase-1 controllers as plugins.
- **Phase 3 (opt)** — MuJoCo + RL, isolated behind an optional Docker target.

## Bridging the core planner (Phase 1.5)

The root `src/GP` simulators (`MPStandardSimulator`, `MPSnakeSimulator`) and
`src/TrajOpt/DevPlanner` model the vehicle dynamics / non-holonomic constraints.
Plan: build them as a shared library, wrap in a thin `rclcpp` node
(`vto_planner_bridge`) that exposes plans as `nav_msgs/Path` and consumes goals —
keeping the © Plaku code behind a clean, swappable interface.
