#!/usr/bin/env bash
# Headless timing diagnostic: is the MPC failure a compute/real-time-factor issue?
# For each controller: measure sim real-time-factor (RTF), the actual /cmd_vel
# publish rate (wall clock), and controller solve-time spike frequency.
set +u
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
source install/setup.bash

measure() {
  CTRL=$1
  echo "================ ${CTRL} (headless) ================"
  ros2 launch vto_bringup maze_astar.launch.py headless:=true controller:="${CTRL}" \
    >/tmp/${CTRL}.log 2>&1 &
  sleep 35   # gazebo + amcl up
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 9.1, y: 14.09, z: 0.0}, orientation: {w: 1.0}}}' \
    >/dev/null 2>&1
  sleep 5    # start driving

  echo "-- real-time factor (gz world stats) --"
  STATS=$(gz topic -l 2>/dev/null | grep -m1 -i stats)
  gz topic -e -t "$STATS" -n 4 2>/dev/null | grep -i real_time_factor | head -4 || echo "  (gz stats unavailable)"

  echo "-- /cmd_vel rate (wall clock, 10 s) --"
  timeout 11 ros2 topic hz /cmd_vel 2>&1 | grep -m1 "average rate" || echo "  (none)"

  echo "-- /controller/compute_ms over 10 s --"
  timeout 11 ros2 topic echo /controller/compute_ms --field data 2>/dev/null > /tmp/${CTRL}_cms.txt
  python3 - "$CTRL" <<'PY'
import sys
c=sys.argv[1]
vals=[float(x) for x in open(f"/tmp/{c}_cms.txt") if x.strip().replace('.','',1).replace('-','',1).isdigit()]
if vals:
    n=len(vals); print(f"  n={n} mean={sum(vals)/n:.2f}ms max={max(vals):.0f}ms "
          f">50ms={sum(1 for v in vals if v>50)} ({100*sum(1 for v in vals if v>50)/n:.1f}%)")
else:
    print("  (no compute_ms data)")
PY
  pkill -f "ros2 launch"; pkill -f gz; pkill -f ruby; pkill -f amcl
  pkill -f mpc_controller; pkill -f pure_pursuit; pkill -f astar; sleep 7
}

measure mpc
measure pursuit
echo "================ interpretation ================"
echo "If RTF>>1 AND mpc /cmd_vel rate << pursuit's (and << 20*RTF) with compute spikes,"
echo "the headless MPC failure is compute/real-time starvation. Else, rethink."
