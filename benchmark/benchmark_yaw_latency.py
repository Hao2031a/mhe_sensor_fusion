#!/usr/bin/env python3
"""Synthetic 55-Hz angular sensor / 100-Hz odom profile. NOT Gazebo ground truth.
Models the former median-of-3 + adaptive LPF + acceleration/jerk shaper
and compares to the new angular bypass, holding sensor rate/noise fixed.
"""
import json
import math
import random
import statistics
from pathlib import Path

DT = 0.01
DURATION = 4.0
SENSOR_DT = 1 / 55


def true_rate(t):
    # start/stop, reversal, steady turn, and double-step maneuvers
    if t < 0.3:
        return 0.0
    if t < 1.2:
        return 1.2
    if t < 1.65:
        return 0.0
    if t < 2.45:
        return -0.9
    if t < 2.8:
        return 0.0
    if t < 3.5:
        return 0.6
    return 0.0


def old_shaper_step(target, value, accel, history):
    history.append(target)
    if len(history) > 3:
        history.pop(0)
    guarded = sorted(history)[1] if len(history) == 3 else target
    ratio = min(1.0, abs(guarded - value) / 0.150)
    blend = ratio * ratio * (3.0 - 2.0 * ratio)
    tau = 0.040 + (0.008 - 0.040) * blend
    filtered = value + DT / (tau + DT) * (guarded - value)
    target_accel = (filtered - value) / DT
    slowing = (value * guarded < 0.0 or abs(guarded) < abs(value))
    limit = 16.0 if slowing else 12.0
    target_accel = max(-limit, min(limit, target_accel))
    max_da = 240.0 * DT
    accel += max(-max_da, min(max_da, target_accel - accel))
    prev = value
    value += accel * DT
    if (guarded - prev) * (guarded - value) <= 0.0:
        value, accel = guarded, 0.0
    return value, accel


def run(seed):
    rng = random.Random(seed)
    t = 0.0
    next_sensor = 0.0
    sensor = 0.0
    reference_yaw = raw_yaw = shaped_yaw = 0.0
    old_value = old_accel = 0.0
    old_history = []
    raw_errors = []
    old_errors = []
    angular_jitter = []
    raw_history = []
    while t < DURATION - DT / 2:
        if t >= next_sensor - 1e-9:
            # Exact sample on the Gazebo simulation-time clock, 55 Hz.
            sensor = true_rate(t) + rng.gauss(0, 0.003)
            next_sensor += SENSOR_DT
        actual = true_rate(t)
        old_value, old_accel = old_shaper_step(sensor, old_value, old_accel, old_history)
        # Mid-step integration over piecewise constant profile. Both measured
        # streams have identical sample-and-hold latency, only filter differs.
        reference_yaw += actual * DT
        raw_yaw += sensor * DT
        shaped_yaw += old_value * DT
        raw_errors.append(raw_yaw - reference_yaw)
        old_errors.append(shaped_yaw - reference_yaw)
        raw_history.append(sensor)
        if len(raw_history) > 1:
            angular_jitter.append(sensor - raw_history[-2])
        t += DT
    def rmse(v):
        return math.sqrt(sum(x*x for x in v)/len(v))
    def p95_abs(v):
        return sorted(abs(x) for x in v)[int(0.95*(len(v)-1))]
    return dict(
        old_yaw_rmse_rad=rmse(old_errors),
        bypass_yaw_rmse_rad=rmse(raw_errors),
        old_p95_abs_yaw_error_rad=p95_abs(old_errors),
        bypass_p95_abs_yaw_error_rad=p95_abs(raw_errors),
        bypass_output_jitter_sample_std=statistics.pstdev(angular_jitter),
    )


def main():
    trials = [run(seed) for seed in range(30)]
    keys = trials[0]
    metrics = {k: statistics.mean(t[k] for t in trials) for k in keys}
    metrics['rmse_improvement_percent'] = 100 * (
        1 - metrics['bypass_yaw_rmse_rad']/metrics['old_yaw_rmse_rad'])
    metrics['p95_improvement_percent'] = 100 * (
        1 - metrics['bypass_p95_abs_yaw_error_rad']/metrics['old_p95_abs_yaw_error_rad'])
    checks = {
        'synthetic_30_seed_yaw_rmse_improvement_ge_40_pct': metrics['rmse_improvement_percent'] >= 40,
        'synthetic_30_seed_yaw_p95_improvement_ge_40_pct': metrics['p95_improvement_percent'] >= 40,
        'unsmoothed_yaw_rmse_below_0_025rad': metrics['bypass_yaw_rmse_rad'] < 0.025,
    }
    out = {'synthetic_only': True, 'gazebo_tested': False,
           'trials': len(trials), 'metrics': metrics,
           'checks': checks, 'pass': all(checks.values())}
    (Path(__file__).with_name('benchmark_yaw_latency.json')).write_text(
        json.dumps(out, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(out, indent=2))
    if not out['pass']:
        raise SystemExit('FAIL: yaw latency regression')

if __name__ == '__main__':
    main()
