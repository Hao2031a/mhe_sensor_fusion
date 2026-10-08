#!/usr/bin/env bash
# Isolated long-duration ROS-node stress test. Run inside the built Docker image.
set -Eeuo pipefail
set +u
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
source /ws/install/setup.bash
set -u
PKG=/ws/src/mhe_sensor_fusion
ARTIFACT_DIR=${CI_ARTIFACT_DIR:-/tmp/mhe-ci-artifacts}
SECONDS_TEST=${MHE_STRESS_SECONDS:-600}
CPU_WORKERS=${MHE_STRESS_CPU_WORKERS:-0}
mkdir -p "$ARTIFACT_DIR"
CPU_PIDS=()
NODE_PID=''
cleanup() {
  set +e
  if [ -n "$NODE_PID" ]; then kill "$NODE_PID" 2>/dev/null || true; wait "$NODE_PID" 2>/dev/null || true; fi
  for pid in "${CPU_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

# Optional CPU contention on the SAME container; opt in for reproducibility.
if (( CPU_WORKERS > 0 && CPU_WORKERS <= 8 )); then
  for ((i=0; i<CPU_WORKERS; i++)); do
    python3 -u -c 'import math; x=1.0
while True:
 x=math.sin(x+0.000001)+math.cos(x+0.001)' > /dev/null 2>&1 &
    CPU_PIDS+=("$!")
  done
elif (( CPU_WORKERS < 0 || CPU_WORKERS > 8 )); then
  echo 'MHE_STRESS_CPU_WORKERS must be in [0,8]' >&2
  exit 2
fi

# Node is isolated on /mhe_ci/odom and must NOT publish production TF.
ros2 run mhe_sensor_fusion mhe_sensor_fusion_node --ros-args \
  --params-file /ws/install/mhe_sensor_fusion/share/mhe_sensor_fusion/config/mhe.yaml \
  -p use_sim_time:=false -p publish_tf:=false \
  -p output_topic:=/mhe_ci/odom > "$ARTIFACT_DIR/mhe_stress_node.log" 2>&1 &
NODE_PID="$!"
python3 "$PKG/ci/long_stress_test.py" \
  --seconds "$SECONDS_TEST" --seed "${MHE_STRESS_SEED:-20261008}" \
  --output "$ARTIFACT_DIR/long_stress.json" \
  2>&1 | tee "$ARTIFACT_DIR/long_stress.log"
# A crash after publishing the last sample is still a failed integration test.
kill -0 "$NODE_PID"
python3 "$PKG/ci/check_long_stress.py" \
  --input "$ARTIFACT_DIR/long_stress.json" \
  --output "$ARTIFACT_DIR/long_stress_gate.json"
echo 'LONG STRESS PASS: ROS node survived; consistency/health reports saved.'
