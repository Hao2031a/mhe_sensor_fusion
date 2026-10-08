#!/usr/bin/env python3
"""Synthetic performance tests for threshold choices. Not a ROS/Ceres benchmark."""
import json
from pathlib import Path
import numpy as np

rng = np.random.default_rng(20261008)
N = 200_000
# Prior innovation S normalized to unit Gaussian: null NIS distribution is chi2(1).
noise = rng.normal(0, 1, N)
nis = noise**2
soft = 9.0
hard = 64.0
null_soft = float(np.mean((nis >= soft) & (nis < hard)))
null_hard = float(np.mean(nis >= hard))
# Single-sample gross wheel/gyro spike, 12 effective sigmas.
spike = rng.normal(12.0, 1.0, N)
spike_hard_rate = float(np.mean(spike**2 >= hard))
# Normal dynamic maneuver already explained by process-model uncertainty.
maneuver = rng.normal(1.2, 1.0, N)
maneuver_hard = float(np.mean(maneuver**2 >= hard))
# Covariance propagation for odometry published independently of Ceres endpoint.
P = np.diag([4e-4, 4e-4, 6.25e-4])
dt = 0.01
v = 0.27
w = 0.3
yaw = 0
min_eigenvalue = 1e9
for _ in range(5000):
    ym = yaw + 0.5*w*dt
    F = np.eye(3)
    F[0, 2] = -v*np.sin(ym)*dt
    F[1, 2] = v*np.cos(ym)*dt
    G = np.array([[np.cos(ym)*dt, -0.5*v*np.sin(ym)*dt*dt],
                  [np.sin(ym)*dt, 0.5*v*np.cos(ym)*dt*dt], [0,dt]])
    U = np.diag([0.03**2,0.03**2])
    P = F @ P @ F.T + G @ U @ G.T + np.eye(3) * 1e-5*dt
    P = 0.5*(P+P.T)
    min_eigenvalue = min(min_eigenvalue, float(np.linalg.eigvalsh(P).min()))
    yaw += w*dt
out = {
    'nominal_false_soft_fraction': null_soft,
    'nominal_false_hard_fraction': null_hard,
    'gross_spike_hard_detection_fraction': spike_hard_rate,
    'maneuver_false_hard_fraction': maneuver_hard,
    'output_covariance_min_eigenvalue': min_eigenvalue,
    'output_covariance_diag_after_50s': np.diag(P).tolist(),
}
out['pass'] = (
    null_soft < 0.005 and null_hard < 1e-4
    and spike_hard_rate > 0.999
    and maneuver_hard < 1e-4
    and min_eigenvalue > 0.0
    and bool(np.isfinite(P).all())
)
print(json.dumps(out, indent=2))
Path(__file__).with_name('benchmark_innovation_gate.json').write_text(json.dumps(out, indent=2))
if not out['pass']:
    raise SystemExit('Innovation gating benchmark FAIL')
