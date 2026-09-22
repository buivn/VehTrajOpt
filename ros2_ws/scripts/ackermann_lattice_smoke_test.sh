#!/usr/bin/env bash
# Smoke test for the Ackermann lattice planner + pure_pursuit_ackermann
# pipeline (docs/39), modeled on phase2_amcl_test.sh's shape but targeting
# maze_ackermann.launch.py (no controller:= arg -- only one follower exists
# for this robot).
set +u
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
source install/setup.bash

ros2 launch vto_bringup maze_ackermann.launch.py headless:=true \
  >/tmp/launch.log 2>&1 &
echo "[waiting 38s for gazebo+lidar+ekf+map_server+amcl+planner+pursuit ...]"
sleep 38

echo "=== nodes up ==="
ros2 node list 2>/dev/null | grep -E "ekf_filter_node|amcl|lattice_ackermann_planner|pure_pursuit_ackermann" || echo "  (none found)"

echo "=== TF map->base_footprint (AMCL+EKF chain) ==="
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
echo "=== planner turning_radius log ==="; grep -m1 "turning_radius=" /tmp/launch.log || echo "  (none)"

echo "=== pure_pursuit_ackermann log (pose source / warnings) ==="
grep -m3 -E "pure_pursuit_ackermann up|TF map->base_footprint unavailable" /tmp/launch.log || echo "  (none)"

echo "=== /ackermann_steering_controller/reference rate (robot driven?) ==="
timeout 6 ros2 topic hz /ackermann_steering_controller/reference 2>&1 | grep -m1 "average rate" || echo "  (no cmd topic)"

echo "=== odometry after 15s of driving (did it move from spawn?) ==="
sleep 15
timeout 4 ros2 topic echo /ackermann_steering_controller/odometry --once 2>/dev/null \
  | grep -A3 "position:" | head -4 || echo "(no odom)"

cp /tmp/launch.log /workspace/ackermann_smoke_launch.log 2>/dev/null

pkill -f "ros2 launch" 2>/dev/null; pkill -f gz 2>/dev/null
pkill -f ruby 2>/dev/null; pkill -f amcl 2>/dev/null
echo "[done]"
