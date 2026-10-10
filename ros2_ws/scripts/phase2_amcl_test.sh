#!/usr/bin/env bash
# Phase 2 end-to-end smoke test (needs GPU for the lidar).
# Brings up the full AMCL stack headless, sends a map-frame goal, and checks the
# map-frame /plan + that the robot is driven (/cmd_vel).
#
#   docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
#     -v $HOME/projects/2026/VehTrajOpt:/workspace vehtrajopt:jazzy \
#     bash -lc '/workspace/ros2_ws/scripts/phase2_amcl_test.sh'
set +u
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
source install/setup.bash

CTRL="${1:-pursuit}"   # follower: pursuit | mpc
echo "[controller=${CTRL}]"
ros2 launch vto_bringup maze_astar.launch.py headless:=true controller:="${CTRL}" \
  >/tmp/launch.log 2>&1 &
echo "[waiting 38s for gazebo+lidar+map_server+amcl+planner+pursuit ...]"
sleep 38

echo "=== follower node running ==="
ros2 node list 2>/dev/null | grep -E "mpc_controller|pure_pursuit" || echo "  (none)"
echo "=== TF map->base_footprint (AMCL) ==="
timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 Translation || echo "(no TF)"

echo "=== send goal (map frame, far corner) ==="
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 9.1, y: 14.09, z: 0.0}, orientation: {w: 1.0}}}' \
  >/dev/null 2>&1
sleep 4

echo "=== /plan frame + length ==="
FR=$(timeout 6 ros2 topic echo /plan --once 2>/dev/null | grep -m1 "frame_id")
N=$(timeout 6 ros2 topic echo /plan --once 2>/dev/null | grep -c "position:")
echo "  plan header ${FR}; n_waypoints=${N}"
echo "=== planner log ==="; grep -m1 "planning: start" /tmp/launch.log || echo "  (none)"
echo "=== /cmd_vel rate (robot driven?) ==="
timeout 6 ros2 topic hz /cmd_vel 2>&1 | grep -m1 "average rate" || echo "  (no cmd_vel)"

pkill -f "ros2 launch" 2>/dev/null; pkill -f gz 2>/dev/null
pkill -f ruby 2>/dev/null; pkill -f amcl 2>/dev/null
echo "[done]"
