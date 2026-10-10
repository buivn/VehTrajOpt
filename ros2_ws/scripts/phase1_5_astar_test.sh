#!/usr/bin/env bash
# Headless smoke test for the A* planner wiring (no Gazebo, no GUI).
# Verifies: lifecycle_manager activates map_server -> planner receives the latched
# /map across the QoS boundary -> a /goal_pose produces a /plan.
#
# Run inside the container:
#   docker run --rm -v $HOME/projects/2026/VehTrajOpt:/workspace vehtrajopt:jazzy \
#     bash -lc '/workspace/ros2_ws/scripts/phase1_5_astar_test.sh'
set +u
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
source install/setup.bash

MAP=install/vto_simulation/share/vto_simulation/maps/maze.yaml
pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT

echo "[1] starting map_server + lifecycle_manager + astar_planner ..."
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=false -p yaml_filename:="$MAP" >/tmp/map_server.log 2>&1 & pids+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false -p autostart:=true \
  -p node_names:="['map_server']" >/tmp/lifecycle.log 2>&1 & pids+=($!)
ros2 run vto_planning astar_planner --ros-args \
  -p use_sim_time:=false -p spawn_x:=-9.1 -p spawn_y:=-14.1 \
  >/tmp/astar.log 2>&1 & pids+=($!)

echo "[2] waiting for map_server to activate + planner to receive /map ..."
sleep 10

echo "[3] publishing a fake /odom (robot at spawn = odom origin) ..."
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry \
  '{header: {frame_id: odom}, child_frame_id: base_footprint,
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' \
  >/dev/null 2>&1 & pids+=($!)
sleep 2

echo "[4] sending a goal (far corner, map frame) ..."
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 9.1, y: 14.09, z: 0.0}, orientation: {w: 1.0}}}'
sleep 2

echo "[5] reading /plan (latched) ..."
timeout 6 ros2 topic echo /plan --once > /tmp/plan.yaml 2>&1
n=$(grep -c "position:" /tmp/plan.yaml || true)

echo "----- planner log -----"; tail -5 /tmp/astar.log
echo "-----------------------"
if [ "$n" -ge 2 ]; then
  echo "PASS: /plan has $n waypoints (map_server -> planner -> /plan chain works)"
  exit 0
else
  echo "FAIL: /plan empty or missing (got $n position entries)"
  echo "--- map_server.log ---"; tail -8 /tmp/map_server.log
  echo "--- lifecycle.log ---"; tail -8 /tmp/lifecycle.log
  exit 1
fi
