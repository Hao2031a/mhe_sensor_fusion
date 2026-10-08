#!/usr/bin/env python3
"""Deterministic synthetic policy regression, NOT Gazebo nor Ceres accuracy."""
import json
import math
import random
from pathlib import Path


def run(seed):
    rng = random.Random(seed)
    # A 100-Hz published trace, a 55-Hz reference sensor with timestamped
    # endpoint yaw, and a modest yaw-rate bias in the continuous propagator.
    dt = 0.01
    sensor_dt = 1.0 / 55.0
    last_sensor_t = 0.0
    yaw_true = 0.0
    yaw_free = 0.0
    yaw_corrected = 0.0
    endpoint_yaw = 0.0
    endpoint_rate = 0.0
    baseline_sq = []
    corrected_sq = []
    max_step = 0.0
    rejected_outliers = 0
    for k in range(1600):
        now = k * dt
        w = 0.6 * math.sin(0.75 * now) + (0.45 if 2.0 < now < 7.0 else 0.0)
        # Same ground-truth motion is known to the synthetic test only.
        yaw_true += w * dt
        if now - last_sensor_t >= sensor_dt - 1e-8:
            last_sensor_t = now
            endpoint_yaw = yaw_true + rng.gauss(0, 0.0015)
            endpoint_rate = w + rng.gauss(0, 0.002)
            # Synthetic occasional solver outlier, rejected by 0.20-rad bound.
            if k in (500, 950):
                endpoint_yaw += 0.8
        # Model biased propagation: use same bias and gyro perturbation on
        # both branches so ONLY the yaw-feedback policy changes.
        measured_w = w + 0.012 + rng.gauss(0, 0.001)
        yaw_free += measured_w * dt
        yaw_corrected += measured_w * dt
        age = now - last_sensor_t
        predicted = endpoint_yaw + endpoint_rate * age
        error = math.remainder(predicted - yaw_corrected, math.tau)
        trusted = 0 <= age <= 0.04 and math.isfinite(error) and abs(error) <= 0.20
        if trusted:
            correction = max(-0.2*dt, min(0.2*dt, 0.15 * error))
            yaw_corrected += correction
            max_step = max(max_step, abs(correction))
        elif k in (500, 950):
            rejected_outliers += 1
        if k > 100:
            baseline_sq.append((yaw_free - yaw_true)**2)
            corrected_sq.append((yaw_corrected - yaw_true)**2)
    return math.sqrt(sum(baseline_sq)/len(baseline_sq)), math.sqrt(sum(corrected_sq)/len(corrected_sq)), max_step, rejected_outliers


def main():
    results = [run(k) for k in range(30)]
    baseline = sum(r[0] for r in results) / len(results)
    corrected = sum(r[1] for r in results) / len(results)
    max_step = max(r[2] for r in results)
    # Hard synthetic regressions: limit correction and require improvement
    # in this test scenario without pretending it transfers to Gazebo.
    checks = {'rate_capped': max_step <= .0020001,
              'lower_synthetic_yaw_rmse': corrected < 0.35 * baseline,
              'reject_large_outliers': all(r[3] >= 2 for r in results)}
    report = {'kind': 'synthetic_policy_model_not_ros_or_gazebo',
              'seeds': len(results), 'sensor_hz': 55, 'output_hz': 100,
              'mean_yaw_rmse_unanchored_rad': baseline,
              'mean_yaw_rmse_opt_in_feedback_rad': corrected,
              'max_correction_rad_per_tick': max_step,
              'checks': checks, 'pass': all(checks.values())}
    path = Path(__file__).with_name('benchmark_timestamp_yaw.json')
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
