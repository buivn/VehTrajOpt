#!/usr/bin/env bash
# Full benchmark sweep: run BOTH controllers through the same goal tour, back to
# back (sequential = clean compute-time metric), writing results/<ctrl>_summary.csv.
#
#   docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
#     -v $HOME/projects/2026/VehTrajOpt:/workspace vehtrajopt:jazzy \
#     bash -lc '/workspace/ros2_ws/scripts/run_bench_sweep.sh 30 2'
set +u
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws
source install/setup.bash

NUM=${1:-30}; REP=${2:-2}; MAXWAIT=${3:-5400}   # per-controller wait cap (s)
rm -rf results; mkdir -p results

for CTRL in pursuit mpc; do
  echo "=== SWEEP ${CTRL} (num_legs=${NUM} repeats=${REP}) @ $(date +%H:%M:%S) ==="
  ros2 launch vto_bench bench.launch.py controller:="${CTRL}" \
    num_legs:="${NUM}" repeats:="${REP}" >/tmp/${CTRL}.log 2>&1 &
  for i in $(seq 1 "${MAXWAIT}"); do
    [ -f "results/${CTRL}.done" ] && break
    sleep 1
  done
  echo "${CTRL}: done=$([ -f results/${CTRL}.done ] && echo yes || echo TIMEOUT) @ $(date +%H:%M:%S)"
  pkill -f "ros2 launch"; pkill -f gz; pkill -f ruby
  pkill -f amcl; pkill -f mpc_controller; pkill -f pure_pursuit; pkill -f benchmark
  sleep 8
done

echo "=== ALL SWEEPS DONE @ $(date +%H:%M:%S) ==="
ls -la results/
for CTRL in pursuit mpc; do
  echo "--- ${CTRL}_summary.csv (reached-leg count) ---"
  awk -F, 'NR>1 && $8==1 {n++} END{print n" legs reached"}' "results/${CTRL}_summary.csv" 2>/dev/null
done
