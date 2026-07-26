# CLAUDE.md — VehTrajOpt

Working agreement and project context for this repo. These instructions OVERRIDE default behavior.

---

## How I want you to work (teaching contract)

I am in **learning mode**. I am learning ROS 2 / robotics as I build this, and I am prepping for **job interviews** on these topics. I want to genuinely understand the structures, frameworks, and pipelines inside a robot model — not just receive working code.

For **every task/feature/step**, before and while writing code, cover these in order:

### 1. General idea / theoretical aspect
- What is the task/mission, framed conceptually.
- The underlying theory: the math, the model, the robotics/control/planning principle at play.
- Where this piece sits in the larger pipeline (perception → planning → control → actuation).

### 2. How to solve it — method selection with justification
- Which method(s) can solve this task.
- **Which method I'm choosing and WHY** — the decisive trade-off for *this* scenario.
- **Why NOT the alternatives** — explicitly compare against the other viable methods (cost, assumptions, real-time feasibility, accuracy, hardware, complexity). This comparison is required, not optional.

### 3. Coding — detailed design explanation
Explain the design before/as you implement, at the level of:
- "We need a **new class** `X` to handle responsibility Y (and why a class, not a free function)."
- "We store this in a **data structure** `Z` (e.g. deque / KD-tree / hash map) because <access pattern / complexity>."
- Key interfaces, ownership, data flow between components, and the complexity/real-time implications.
- Reference concrete `file:line` locations when discussing existing code.

### Pace & format rules
- **Go SLOW.** One package or one function per step. No large multi-file bursts.
- Write a **markdown doc in `docs/`** for every step (numbered, e.g. `07-...md`) so I can re-read it later.
- Every doc includes an **_Interview angle_** section with likely interview Q&A on that topic.
- **Check in on pace/depth after each doc** before continuing. I drive the tempo.

---

## Project overview

VehTrajOpt is being grown from an existing C++ research planner into a **ROS 2 robotics stack**.

**First target robot:** differential-drive mobile robot. **Purpose:** research, but keep the hardware door open.

### Repo layout
- **`src/`** — existing core: plain-CMake `Runner` project. Sampling-based planner + trajectory optimizer with OpenGL/GLUT viz. Robot dynamics / non-holonomic models live here:
  - `src/GP/MPStandardSimulator` (car-like), `src/GP/MPSnakeSimulator` (snake), `src/GP/MPSimulator` (base), `src/TrajOpt/DevPlanner`.
- **`ros2_ws/`** — new ROS 2 layer (colcon), kept separate so it never collides with the root plain-CMake build. Wraps the core planner behind a clean interface rather than absorbing it.
  - Packages: `vto_description`, `vto_simulation`, `vto_control`, `vto_bringup`.
- **`docker/`** — Jazzy + Gazebo Harmonic + GPU (nvidia-container-toolkit).
- **`docs/`** — numbered learning docs (00–06 so far) + `commands.md`.

### Copyright constraint (important)
Core `src/` files carry `Copyright (C) 2023 Erion Plaku — All Rights Reserved, do not distribute` (my advisor). I have his **verbal** permission to reuse the code for new projects. Repo is **private for now**.
- Keep his headers untouched. No LICENSE changes.
- If this ever goes public: need **written** distribution permission + isolate his code + separate LICENSE/NOTICE.

### Stack decisions
- ROS 2 **Jazzy** / Ubuntu 24.04 / Gazebo **Harmonic** (not Classic).
- `robot_localization` **EKF** for KF/fusion.
- **Nav2** (not MoveIt) is the interaction layer for wheeled robots — MoveIt only fits snake-as-manipulator kinematics.
- GPU is mainly for MPPI rollouts + MuJoCo/RL, not classical MPC.

### Roadmap
- **Phase 0** — Docker + diffbot spawns/drives in Gazebo. ✅ done + verified.
- **Phase 1** — control: Pure Pursuit → MPC (CasADi/acados) → MPPI, common `Path → cmd_vel` interface. Pure Pursuit ✅.
- **Phase 1.5** — `vto_planner_bridge` wrapping core C++ as a shared lib publishing `nav_msgs/Path`.
- **Phase 2** — sensors + EKF fusion + Nav2.
- **Phase 3** — MuJoCo / RL (optional).

---

## Current progress

- **Phase 0 (Docker + diffbot in Gazebo)** — ✅ verified via `ros2_ws/scripts/phase0_smoke_test.sh` (spawns, controllers activate, drives 0→1.82 m, odom integrates). Controller topics remapped to standard `/cmd_vel` + `/odom`.
- **Phase 1 Pure Pursuit** — ✅ verified. `vto_control` has `pure_pursuit` + `path_publisher` nodes; `vto_bringup/pursuit_sim.launch.py` runs sim+pursuit; `scripts/phase1_pursuit_test.sh` passes (follows quarter-circle arc to within 0.16 m of goal (2,2)). Doc 05.
- **Maze env** — ✅ loads. `vto_simulation/scripts/generate_maze.py` procedurally generates `worlds/maze.sdf` (20×30 m, 1.5 m corridors, seeded recursive-backtracker, 191 wall boxes, `--origin center` default; robot spawns ~(-9.1,-14.1)). `vto_bringup/maze.launch.py` loads it. Doc 06. **Needs a planner** to actually solve — a fixed test path won't work in a maze.

### Still TODO / next options
- I still need to eyeball the GUI once: `ros2 launch vto_bringup sim.launch.py` and `pursuit_sim.launch.py`.
- Deferred Phase-1 enhancement: **curvature-regulated speed** (comfort constraint, ties to advisor's work).
- **Next direction (undecided):** MPC controller, **or** Phase 1.5 C++ planner bridge, **or** Nav2. Planner bridge connects directly to the maze + advisor's code; MPC is the stronger standalone interview topic.

---

## Environment & known gotchas

**Env:** Ubuntu 24.04, driver 580 (CUDA 13.0), docker-ce. I run plain `docker`; the **agent shell needs `sg docker -c '...'`**. Image `vehtrajopt:jazzy`, base `nvidia/cuda:12.6.3-runtime-ubuntu24.04`.

**Jazzy gotchas (verified real):**
- `diff_drive_controller` uses **TwistStamped on `/diff_drive_controller/cmd_vel`** + odom on `/diff_drive_controller/odom` (not bare `/cmd_vel`/`/odom`, not plain `Twist`). `use_stamped_vel:false` is ignored.
- `robot_description` `Command()` must be wrapped in `ParameterValue(..., value_type=str)`.
- ROS `setup.bash` isn't `set -u`-clean → use `set +u` around sourcing.

**Phase 1 gotchas (verified real):**
- A hand-made `ament_python` pkg MUST have `setup.cfg` with `install_scripts=$base/lib/<pkg>`, else executables land in `bin/` and launch can't find them.
- Pure pursuit `_lookahead_point` must search **FORWARD** from the closest path index, else the carrot flips behind and the robot oscillates.
- In-container colcon runs as **root** → root-owned `build/install/__pycache__` in the mounted repo (gitignored; `sudo rm -rf` to clean; use `ast.parse`, not `py_compile`, on the host).

---

## Git
- Working branch: `ros2-jazzy-phase0-1-maze`. Main branch: `main`.
- Commit/push only when I ask.
