#!/usr/bin/env bash
set -e

# Source ROS 2 and the workspace overlay if it has been built.
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi

exec "$@"
