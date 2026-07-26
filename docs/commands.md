# Commands Cheatsheet

Every command to build, run, drive, test, and debug VehTrajOpt. Copy-paste
friendly. For the *why* behind the setup, see
[doc 04](04-environment-setup.md).

> **Note:** these are the **user's** commands (you're in the `docker` group, so
> plain `docker`). If a script/agent runs Docker from a shell that predates the
> group change, prefix with `sg docker -c "..."`.

---

## 0. One-time host setup

See [doc 04](04-environment-setup.md) for the full walkthrough. Summary:

```bash
# GPU driver (needs reboot)
sudo apt install -y nvidia-driver-580 && sudo reboot
nvidia-smi                                    # verify driver 580, CUDA 13.0

# Docker Engine + group (see doc 04 for the repo setup block)
sudo usermod -aG docker $USER                 # then log out/in

# nvidia-container-toolkit, then smoke test
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

---

## 1. Build the Docker image

```bash
cd ~/projects/2026/VehTrajOpt/docker
docker compose build                          # first build ~10 GB, slow
docker images vehtrajopt:jazzy                # confirm it exists
```

---

## 2. Build the ROS 2 workspace (inside the container)

```bash
# start an interactive container (repo mounted at /workspace)
xhost +local:root                             # allow GUI (once per login)
cd ~/projects/2026/VehTrajOpt/docker
docker compose run --rm vto

# --- now inside the container ---
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

One-shot build without an interactive shell:
```bash
docker run --rm -v ~/projects/2026/VehTrajOpt:/workspace vehtrajopt:jazzy \
  bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros2_ws \
            && rosdep install --from-paths src --ignore-src -r -y \
            && colcon build --symlink-install'
```

> `--symlink-install` symlinks launch/config/python files, so edits to those take
> effect **without** rebuilding. C++ changes still need `colcon build`.

---

## 3. Launch

Inside the container, after sourcing `install/setup.bash`:

```bash
# --- Phase 0: empty world + diffbot + controllers ---
ros2 launch vto_bringup sim.launch.py                       # Gazebo + rviz
ros2 launch vto_simulation gazebo.launch.py headless:=true  # server only, no GUI

# --- Phase 1: sim + Pure Pursuit following a test path ---
ros2 launch vto_bringup pursuit_sim.launch.py               # GUI: watch it follow the arc
ros2 launch vto_bringup pursuit_sim.launch.py headless:=true

# --- Maze world (20x30, 1.5 m corridors); robot spawns bottom-left ---
ros2 launch vto_bringup maze.launch.py                      # GUI
ros2 launch vto_bringup maze.launch.py headless:=true
# regenerate the maze (then colcon build vto_simulation):
python3 src/vto_simulation/scripts/generate_maze.py --seed 12 \
  --out src/vto_simulation/worlds/maze.sdf
```

---

## 4. Drive the robot

New shell into the *same* running container:
```bash
docker exec -it vehtrajopt bash
source /workspace/ros2_ws/install/setup.bash
```

> **Jazzy note:** `diff_drive_controller` takes **`geometry_msgs/TwistStamped`**
> on **`/diff_drive_controller/cmd_vel`** (not a plain `Twist` on
> `cmd_vel_unstamped`). Odometry is on **`/diff_drive_controller/odom`**.

Then either:
```bash
# one-off velocity command (verified working) — forward 0.4 m/s, turn 0.3 rad/s
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.4}, angular: {z: 0.3}}}"

# keyboard teleop — needs stamped:=true so it emits TwistStamped
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel
```

---

## 5. Introspect / debug

```bash
# controllers loaded and active?
ros2 control list_controllers

# is sim time flowing (bridge alive)?
ros2 topic hz /clock

# is odometry updating? (drive the robot, watch x/y change)
ros2 topic echo /odom --field pose.pose.position

# wheels reporting? (broadcaster alive)
ros2 topic echo /joint_states --once

# the whole topic graph
ros2 topic list
ros2 node list

# TF tree: is odom -> base_footprint -> wheels present?
ros2 run tf2_tools view_frames        # writes frames.pdf
ros2 run tf2_ros tf2_echo odom base_footprint

# expand the URDF / validate the tree
xacro src/vto_description/urdf/mobile/diffbot.urdf.xacro | check_urdf /dev/stdin
```

---

## 6. Automated tests (headless)

Run **inside the container**, after `source install/setup.bash`:

```bash
# Phase 0 — spawn, activate controllers, drive, assert /odom moved >0.2 m
bash scripts/phase0_smoke_test.sh

# Phase 1 — sim + Pure Pursuit follows the arc, assert robot reaches ~(2,2)
bash scripts/phase1_pursuit_test.sh
```

Or as a **one-shot from the host** (fresh throwaway container, GPU on):

```bash
docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v ~/projects/2026/VehTrajOpt:/workspace vehtrajopt:jazzy \
  bash /workspace/ros2_ws/scripts/phase1_pursuit_test.sh
```

Exit code 0 = PASS. Only run **one** at a time — two Gazebo servers starve the GPU.

---

## 7. Cleanup

```bash
# stop containers
docker ps                              # list running
docker stop vehtrajopt

# remove root-owned build artifacts (created by in-container colcon)
sudo rm -rf ~/projects/2026/VehTrajOpt/ros2_ws/{build,install,log}

# free image disk if rebuilding from scratch
docker image rm vehtrajopt:jazzy
docker system df                       # see docker disk usage
```

---

## Common gotchas

| Symptom | Cause / fix |
|---|---|
| `permission denied ... docker.sock` | not in `docker` group in this shell → log out/in, or `sg docker -c "..."` |
| `image ... not found` on build | base tag doesn't exist — verify on Docker Hub (we hit this with CUDA 12.4) |
| rviz "extrapolation" errors | a node missing `use_sim_time:=true` |
| robot spawns but won't move | check `cmd_vel` remap; controllers `active`? (`ros2 control list_controllers`) |
| Gazebo GUI black / no window | `xhost +local:root` not run, or `DISPLAY` not passed |
| build artifacts owned by root | in-container colcon runs as root; `sudo rm -rf` them (they're gitignored) |

---

## Packaging: build → install → run (and deployment)

`colcon build` does **build *and* install in one step** into three dirs:
- `build/` — intermediate compile artifacts (throwaway)
- `install/` — the **install space** you actually run from
- `log/` — build logs

Install layout per package (this is where things land):
```
install/<pkg>/lib/<pkg>/                         # node executables
install/<pkg>/lib/                               # shared libs (.so)
install/<pkg>/lib/python3.x/site-packages/<pkg>/ # python modules
install/<pkg>/share/<pkg>/                       # launch, config, urdf, meshes, msgs
```
`source install/setup.bash` sets **`AMENT_PREFIX_PATH`** (ament index lookup) + `PATH`,
`LD_LIBRARY_PATH`, `PYTHONPATH` so ROS finds it. Default prefix is the workspace
`./install` (not `/var`); core ROS lives in `/opt/ros/jazzy`. In our container the repo
is bind-mounted, so `install/` is both `/workspace/ros2_ws/install` (container) and
`ros2_ws/install` (host, gitignored).

- **`--symlink-install`** symlinks source into `install/` (edit launch/py/config, no
  rebuild) — **dev only**. For a robot use a plain `colcon build` (real copies).
- **Deploy to a Jetson:** ship the workspace + a `systemd` unit that sources
  `install/setup.bash`; or build a `.deb` (bloom) into `/opt/ros/<distro>`; or a container.
- **Container hardware access:** `--runtime nvidia` (GPU), `--device /dev/ttyUSB0` etc.
  (lidar/camera/CAN), usually `--network host` (DDS discovery).
