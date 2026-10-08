#!/usr/bin/env python3
"""Check that CI regression gates actually return a failing exit code."""
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as temp:
    dest = Path(temp)
    (dest / 'benchmark').mkdir()
    for name in ('benchmark_upgrade', 'benchmark_stability',
                 'benchmark_stability_mc', 'benchmark_innovation_gate',
                 'benchmark_consistency_mc', 'benchmark_yaw_latency',
                 'benchmark_solver_fallback', 'benchmark_timestamp_yaw',
                 'benchmark_block_schur', 'benchmark_rank_aware_prior',
                 'benchmark_so2_yaw','benchmark_gyro_increment'):
        shutil.copy2(root / 'benchmark' / f'{name}.json',
                     dest / 'benchmark' / f'{name}.json')
    check = root / 'ci/check_regressions.py'
    args = [sys.executable, str(check), '--package', str(dest)]
    valid = subprocess.run(args, capture_output=True, text=True)
    if valid.returncode != 0:
        raise RuntimeError('Positive test failed: ' + valid.stdout + valid.stderr)
    bad_file = dest / 'benchmark/benchmark_stability.json'
    bad = json.loads(bad_file.read_text())
    bad['overall_pass'] = False
    bad_file.write_text(json.dumps(bad))
    invalid = subprocess.run(args, capture_output=True, text=True)
    if invalid.returncode == 0:
        raise RuntimeError('Negative test did not fail CI')
print('PASS: positive and negative regression gate controls')

# Test the ROS environment bootstrap with a realistic unset AMENT variable.
# This runs without ROS installed by using fake generated setup files.
with tempfile.TemporaryDirectory() as temp:
    tmp = Path(temp)
    ros_setup = tmp / 'ros_setup.bash'
    ws_setup = tmp / 'ws_setup.bash'
    ros_setup.write_text('if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then :; fi\nROS_SETUP_OK=yes\n')
    ws_setup.write_text('if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then :; fi\nOVERLAY_OK=yes\n')
    script = (root / 'ci/run_ci.sh').read_text()
    bootstrap = script.split('PKG=/ws/src/mhe_sensor_fusion', 1)[0]
    bootstrap = bootstrap.replace('/opt/ros/${ROS_DISTRO:-kilted}/setup.bash', str(ros_setup))
    bootstrap = bootstrap.replace('/ws/install/setup.bash', str(ws_setup))
    bootstrap += '\n[[ $- == *u* ]] && [[ "$ROS_SETUP_OK" == yes ]] && [[ "$OVERLAY_OK" == yes ]]\n'
    env = dict(__import__('os').environ)
    env.pop('AMENT_TRACE_SETUP_FILES', None)
    proc = subprocess.run(['bash', '-c', bootstrap], env=env, capture_output=True, text=True)
    if proc.returncode:
        raise RuntimeError('ROS bootstrap fails with unset AMENT_TRACE_SETUP_FILES: ' + proc.stderr)
print('PASS: ROS setup tolerates unset AMENT_TRACE_SETUP_FILES and restores nounset')
