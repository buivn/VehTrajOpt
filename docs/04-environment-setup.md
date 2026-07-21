# 04 — Environment Setup: Docker + GPU + GUI

How we get Phase 0 actually *running*: a container with **ROS 2 Jazzy + Gazebo
Harmonic**, using the **NVIDIA GPU** for rendering/compute, and showing the
**Gazebo/rviz GUI** on your screen. This doc records the real setup on this
machine (Quadro T2000, Ubuntu 24.04) and the lessons each step teaches.

Target machine (2026-07): Ubuntu 24.04.4, Quadro T2000 (Turing, CC 7.5), NVIDIA
driver 535 → upgrading to 580.

---

## The 4 setup steps

```
1. GPU driver         535 → 580   (host, needs reboot)
2. Docker Engine      snap → docker-ce   (host)
3. nvidia-container-toolkit         (host, lets containers see the GPU)
4. Build + run the image            (docker/)
```

Steps 1–3 are **host** setup (one-time, need sudo). Step 4 is our project.

---

## Step 1 — GPU driver, and the CUDA-vs-driver lesson

### The three "CUDA" layers (a classic confusion)

| Layer | What it is | Gated by |
|---|---|---|
| **GPU hardware** | the silicon (T2000 = Turing, CC 7.5) | fixed |
| **Driver / libcuda** | kernel driver talking to the GPU | **the thing that caps CUDA version** |
| **CUDA runtime/toolkit** | libraries the app uses | ships *inside* the container |

Key insight: **the container carries its own CUDA runtime; the host only needs a
new-enough driver.** You do *not* install CUDA on the host for containerized work.

### The version rule

Each CUDA version needs a minimum driver:

| CUDA | min driver |
|---|---|
| 12.2 | ≥ 535 |
| 12.4 | **≥ 550** |
| 12.6 | ≥ 560 |

Our image is CUDA **12.6.3** → needs driver ≥ 560. This box had **535** (caps at
12.2), and NVIDIA's CUDA images embed a `NVIDIA_REQUIRE_CUDA` gate that makes the
container **refuse to start** on too-old a driver. Fix = upgrade the driver (we
went to 580, max CUDA 13.0).

> **Gotcha we hit:** the Dockerfile first pinned `nvidia/cuda:12.4.1-...-ubuntu24.04`
> and the build failed with `not found`. NVIDIA never published a CUDA **12.4**
> image for **Ubuntu 24.04** — the earliest 24.04 tag is **12.5.1**. Lesson: a
> tag string is not proof the tag exists; verify against the registry
> (`curl https://hub.docker.com/v2/repositories/nvidia/cuda/tags?name=runtime-ubuntu24.04`).
> We settled on `12.6.3-runtime-ubuntu24.04`.

### Why upgrade the driver, not the GPU

The T2000 (Turing) is fully supported by driver branches 550/580/595. So CUDA 12.4
is a **software update**, not a hardware limit.

```bash
sudo apt update
sudo apt install -y nvidia-driver-580   # what we used; `sudo ubuntu-drivers install` picks 595
sudo reboot
nvidia-smi                              # verify: Driver 580, "CUDA Version: 13.0" (>= 12.4 ✓)
```

> `nvidia-smi`'s "CUDA Version" = the **max** the driver supports, not what's
> installed. Driver 580 → max CUDA 13.0, comfortably above our image's 12.4.
> (`ubuntu-drivers install` would have installed 595-open — either works.)

---

## Step 2 — Docker Engine, and the snap-confinement lesson

This box first had **snap Docker** (`/snap/bin/docker`). Symptoms that reveal it:
`docker.service does not exist` (the snap unit is `snap.docker.dockerd.service`),
and binaries under `/snap/bin`.

### Why snap Docker is wrong for this project

Snap packages run **strictly confined** — a snap can only bind-mount paths under
`$HOME`. But GUI + GPU need mounts *outside* home:

- `/tmp/.X11-unix` (the X11 socket — how Gazebo/rviz draw on screen) → **blocked**
  by confinement → no GUI.
- GPU passthrough + arbitrary mounts get fragile under confinement.

So for **GUI + GPU + bind mounts**, use the unconfined **Docker Engine**
(`docker-ce`) from Docker's official apt repo — the standard ROS/Gazebo setup.

```bash
sudo snap remove docker                       # remove the confined one

sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo systemctl enable --now docker            # docker.service now exists
```

### The docker group / socket lesson

Docker talks over `/var/run/docker.sock`, owned `root:docker` (mode `660`). To use
Docker without `sudo`, join the `docker` group:

```bash
sudo usermod -aG docker $USER     # one-time; needs sudo (unavoidable)
newgrp docker                     # apply to current shell (or log out/in)
docker ps                         # works with no sudo
```

- Group membership only applies to shells started *after* the change — a full
  **logout/login** (or reboot) applies it everywhere.
- **Security note (interview-relevant):** the `docker` group is effectively
  **root** — you can mount the host FS as root in a container. Fine on a personal
  dev box; reconsider on shared/production machines.
- **True no-sudo alternative:** *rootless Docker* runs the daemon as your user —
  but complicates GPU passthrough, so we don't use it here.

---

## Step 3 — nvidia-container-toolkit (GPU into the container)

Docker alone can't see the GPU. The **nvidia-container-toolkit** injects the host
driver + device nodes into containers and registers an NVIDIA runtime with Docker.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker   # wires the runtime into dockerd
sudo systemctl restart docker

# smoke test: the GPU should appear INSIDE a container
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

If that last command prints your GPU table, the GPU crosses the container boundary
— the whole point.

> This is separate from CUDA: the toolkit passes the **driver**; CUDA libraries
> come from the image. That's why the host never installs CUDA.

---

## Step 4 — Build and run Phase 0

```bash
xhost +local:root                 # let containers talk to your X server (GUI)
cd ~/projects/2026/VehTrajOpt/docker
docker compose build              # first build downloads ROS/Gazebo (several GB, slow)
docker compose run --rm vto

# inside the container:
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch vto_bringup sim.launch.py
```

**Success = Phase 0 verified:** Gazebo opens, the diffbot sits on the ground,
rviz shows the model + TF. Drive it (new shell into the same container):

```bash
docker exec -it vehtrajopt bash
source /workspace/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

---

## Verification checklist

| Check | Command | Expected |
|---|---|---|
| Driver upgraded | `nvidia-smi` | Driver 580, CUDA 13.0 (≥ 12.4) |
| Docker Engine | `docker version` | Client **and** Server both print |
| No-sudo Docker | `docker ps` | table, no permission error |
| GPU in container | `docker run --rm --gpus all ubuntu:24.04 nvidia-smi` | GPU table |
| GUI works | `ros2 launch vto_bringup sim.launch.py` | Gazebo + rviz windows open |

---

## Interview angle

**Q: Do you install CUDA on the host to run GPU containers?**
No. The container ships its own CUDA runtime; the host only needs a driver new
enough for that CUDA version (+ nvidia-container-toolkit). CUDA version is gated by
the **driver**, not the host toolkit.

**Q: What does nvidia-container-toolkit actually do?**
It injects the host GPU driver, libraries, and device nodes into the container and
registers an NVIDIA runtime with Docker, so `--gpus all` works. Without it,
containers can't see the GPU.

**Q: Why not snap Docker for a GUI/GPU workload?**
Snap strict confinement blocks bind-mounts outside `$HOME` (e.g. `/tmp/.X11-unix`
for the GUI) and complicates GPU passthrough. Use unconfined Docker Engine.

**Q: How does a container draw a GUI on the host?**
Mount the host X11 socket (`/tmp/.X11-unix`), pass `DISPLAY`, and authorize the
container (`xhost +local:root`). The container's GUI apps talk to the host X
server. (Modern alternative: Wayland or a web-based viewer.)

**Q: Why is being in the `docker` group a security consideration?**
It's root-equivalent — you can bind-mount the host filesystem as root inside a
container. Acceptable on a personal machine, risky on shared systems.

---

## Where this maps in our repo

- [`docker/Dockerfile`](../docker/Dockerfile) — the CUDA 12.4 + Jazzy + Gazebo image
- [`docker/docker-compose.yml`](../docker/docker-compose.yml) — GPU request, X11 +
  repo mounts, host networking
- [`docker/entrypoint.sh`](../docker/entrypoint.sh) — sources ROS + the workspace
- **Next:** once verified, we start **Phase 1 — Pure Pursuit** (code + doc 05).
