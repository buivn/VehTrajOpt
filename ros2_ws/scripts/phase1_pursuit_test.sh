#!/usr/bin/env bash
# Phase 1 test: launch sim + Pure Pursuit + test path, assert the robot reaches
# the goal (~2, 2) at the end of the arc.
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
set -u

GOAL_X=2.0; GOAL_Y=2.0; TOL=0.5

echo "[pp] launching sim + pure pursuit (headless)..."
ros2 launch vto_bringup pursuit_sim.launch.py headless:=true >/tmp/pp.log 2>&1 &
LAUNCH_PID=$!
cleanup() { kill "$LAUNCH_PID" 2>/dev/null; pkill -f "parameter_bridge" 2>/dev/null; pkill -f "gz sim" 2>/dev/null; }
trap cleanup EXIT

# wait for controllers (timeout each query so a dead launch can't hang the poll)
echo "[pp] waiting for controllers..."
for i in $(seq 1 60); do
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "[pp] FAIL: launch process died early"; tail -30 /tmp/pp.log; exit 1; }
  timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" && break
  sleep 1
done
if ! timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active"; then
  echo "[pp] FAIL: controllers never activated"; tail -30 /tmp/pp.log; exit 1
fi
echo "[pp] controllers active; following path..."

read_odom() {  # $1 = x|y
  timeout 10 ros2 topic echo /odom --field pose.pose.position.$1 --once 2>/dev/null \
    | tr -d ' ' | grep -E '^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$' | tail -1
}

# give Pure Pursuit time to drive the ~3 m arc
sleep 25

X=$(read_odom x); Y=$(read_odom y)
echo "[pp] final pose: x=${X:-<none>} y=${Y:-<none>}  (goal ${GOAL_X},${GOAL_Y})"

if python3 -c "import sys,math; d=math.hypot((${X:-0})-$GOAL_X,(${Y:-0})-$GOAL_Y); print(f'dist={d:.3f}'); sys.exit(0 if d < $TOL else 1)"; then
  echo "[pp] PASS: reached goal within $TOL m"
  exit 0
else
  echo "[pp] FAIL: did not reach goal (tol $TOL m)"
  exit 1
fi
