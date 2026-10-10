#!/usr/bin/env bash
# Phase 0 smoke test: headless-launch the sim, drive the robot, assert /odom moved
set +u

source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

set -u

echo "[smoke] launching headless sim ..."
ros2 launch vto_simulation gazebo.launch.py headless:=true >/tmp/launch.log 2>&1 &
LAUNCH_PID=$!

# always clean up the launch on exit (pass or fail)
cleanup() { kill "$LAUNCH_PID" 2>/dev/null; pkill -f "parameter_bridge" 2>/dev/null; pkill -f "gz sim" 2>/dev/null;}
trap cleanup EXIT

#1. wait up to 60s for the diff_drive_controller to become active
echo "[smoke] waiting for controllers..."
for i in $(seq 1 60); do 
    ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" && break 
    sleep 1
done
if ! ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active"; then
    echo "[smoke] FAIL: diff_drive_controller never became active"
    ros2 control list_controllers 2>/dev/null; cat /tmp/launch.log | tail -30
    exit 1
fi 
echo "[smoke] controllers active"

# helper: read one odom-x value, filtering out DDS warnings / "---" separators
# (topics remapped to the standard /odom, /cmd_vel by the controller spawner)
read_odom_x() {
  timeout 10 ros2 topic echo /odom \
    --field pose.pose.position.x --once 2>/dev/null \
    | tr -d ' ' | grep -E '^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$' | tail -1
}

#2. read initial odom x
X0=$(read_odom_x)
echo "[smoke] initial x=${X0:-<none>}"

#3. drive forward ~4 s  (Jazzy diff_drive_controller = TwistStamped on cmd_vel)
echo "[smoke] driving forward..."
timeout 4 ros2 topic pub -r 20 /cmd_vel \
    geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}}}" >/dev/null 2>&1

#4. read final odom x
X1=$(read_odom_x)
echo "[smoke] final x = ${X1:-<none>}"

#5. assert it moved > 0.2 m
if python3 -c "import sys; sys.exit(0 if (${X1:-0})-(${X0:-0}) > 0.2 else 1)"; then
    echo "[smoke] PASS; robot moved ${X0} -> ${X1}"
    exit 0
else
    echo "[smoke] FAIL; robot did not move enough (${X0}) -> ${X1})"
    exit 1
fi