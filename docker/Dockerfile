# ROS 2 Jazzy + Gazebo Harmonic + GPU, on Ubuntu 24.04
# Base ships the CUDA runtime; nvidia-container-toolkit exposes the host GPU.
#
# Tag notes:
#  - NVIDIA's earliest CUDA image for Ubuntu 24.04 is 12.5.1 (no 12.4/24.04 exists).
#  - 12.6.3 needs driver >= 560; this host runs 580 (max CUDA 13.0), so it passes.
#  - `runtime` (not `devel`) = CUDA libs but no nvcc. Fine for Phase 0/Gazebo.
#    Switch to `12.6.3-devel-ubuntu24.04` when Phase 1 compiles custom CUDA (MPPI).
FROM nvidia/cuda:12.6.3-runtime-ubuntu24.04

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# --- Locale + basics -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales curl gnupg2 lsb-release software-properties-common \
      build-essential cmake git python3-pip \
      # existing core-planner deps (OpenGL/GLUT + libpng)
      freeglut3-dev libpng-dev libgl1-mesa-dev \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --- ROS 2 Jazzy apt source ------------------------------------------------
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# --- ROS 2 desktop + Gazebo Harmonic + control/nav/localization stacks -----
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-ros-gz \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-robot-localization \
      ros-jazzy-nav2-bringup ros-jazzy-navigation2 \
      ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
      python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# NVIDIA runtime hints (used with nvidia-container-toolkit)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN groupadd -g 992 render || true

# --- Python deps for MPC ---------------------------------------------------
# CasADi (nonlinear MPC modeling + autodiff; ships IPOPT). Late layer so a
# rebuild only re-runs this step. Ubuntu 24.04 pip is PEP-668 externally-managed
# -> --break-system-packages. numpy/scipy already come from the ROS desktop apt.
RUN pip install --no-cache-dir --break-system-packages casadi

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
