#!/usr/bin/env python3
"""Hard regression gate for long duration ROS/Ceres MHE tests.

Portable gate: liveness/integrity + coarse scheduling on shared hosts.
Strict gate: deadline for a reproducible self-hosted runner.
Consistency gate: opt-in statistical thresholds for synthetic pose NEES;
NEES/NIS are always recorded, even when calibration gating is off.
"""
import argparse
import json
import math
import os
from pathlib import Path


def check_report(r, strict=False, consistency_gate=False):
    seconds = float(r.get('configured_seconds', 0.))
    elapsed = float(r.get('elapsed_seconds', 0.))
    cov = r.get('covariance_consistency', {})
    worker = r.get('covariance_worker', {})
    solver = r.get('solver_ms', {})
    callback = r.get('callback_ms', {})
    nis = r.get('innovation_consistency', {})
    target_count = max(1, int(seconds * (75 if strict else 60)))
    max_miss = .001 if strict else .10
    checks = {
        'actual_ros_node_output': r.get('kind') == 'ros_node_synthetic_ground_truth_stress',
        'duration_completed': elapsed >= seconds - .1,
        'odom_rate_sufficient': r.get('odom_count', 0) >= target_count,
        'output_all_finite': r.get('odom_nonfinite_count', 1) == 0,
        'output_stamps_strictly_increasing': r.get('odom_nonmonotonic_stamp_count', 1) == 0,
        'no_pose_covariance_negative_eigenvalues': cov.get('invalid_or_negative_eigen_count', 1) == 0,
        'truth_aligned_for_consistency': cov.get('pose_error_nees', {}).get('count', 0) >= max(20, int(seconds * 3)),
        'nominal_nees_available': cov.get('nominal_nees_sample_count', 0) >= max(10, int(seconds * 1)),
        'approximate_nis_available': all(c.get('count', 0) >= max(10, int(seconds * 3))
                                         for c in nis.get('approximate_prefit_nis', []))
        and len(nis.get('approximate_prefit_nis', [])) == 4,
        'callbacks_profiled': callback.get('count', 0) >= max(50, int(seconds * 60)),
        'solver_profiled': solver.get('count', 0) >= max(20, int(seconds * 30)),
        'worker_completed': worker.get('completed', 0) >= 1,
        'worker_status_valid': worker.get('valid', 0) == 1,
        'worker_failure_count_zero': worker.get('failed', 1) == 0,
        'callback_misses_bounded': r.get('callback_deadline_miss_fraction', 1.) <= max_miss,
        'solver_misses_bounded': r.get('solver_deadline_miss_fraction', 1.) <= max_miss,
        'no_excessive_rollbacks': r.get('solver_rollbacks', float('inf')) <= max(2, int(seconds * .02)),
        'maximum_odom_gap_bounded': r.get('max_odom_gap_ms', float('inf')) <= (100 if strict else 400),
        'callback_p99_bounded': callback.get('p99') is not None and callback['p99'] < (6. if strict else 25.),
    }
    if seconds >= 18:
        checks['encoder_spike_injected'] = r.get('injections', {}).get('encoder_spike', 0) > 0
    if seconds >= 23:
        checks['dropout_scenarios_injected'] = (
            r.get('injections', {}).get('imu_dropout_ticks', 0) > 0 and
            r.get('injections', {}).get('wheel_dropout_ticks', 0) > 0)
    if seconds >= 25:
        checks['out_of_order_injected'] = r.get('injections', {}).get('imu_out_of_order', 0) > 0
    if strict:
        checks['solver_p99_below_5ms'] = solver.get('p99') is not None and solver['p99'] < 5.
        checks['cov_worker_age_under_120'] = worker.get('age_solves', float('inf')) < 120
    if consistency_gate:
        # Run only on independently timestamp-aligned, truly evaluated data.
        # The report MUST include state-wise consistency and 30-second-cycle
        # bootstrap metadata. Do not accept legacy NEES-only reports as calibrated.
        nees = cov.get('nominal_nees', {})
        coverage = cov.get('nominal_95pct_coverage', 0.)
        state_nees = cov.get('nominal_state_nees', {})
        state_coverage = cov.get('nominal_state_95pct_coverage', {})
        blocks = cov.get('independent_cycle_bootstrap', {})
        alignment = cov.get('alignment_bracket_ms', {})
        checks['truth_not_extrapolated'] = cov.get('alignment_extrapolation_allowed', True) is False
        checks['aligned_timestamp_brackets_bounded'] = (
            alignment.get('count', 0) >= max(10, int(seconds)) and
            alignment.get('p99') is not None and alignment['p99'] <= 30.0)
        checks['independent_trajectory_blocks_available'] = (
            blocks.get('valid', False) and blocks.get('independent_blocks', 0) >= 5)
        # Conservative diagnostic envelopes: intentionally avoid pretending
        # that successive 100-Hz poses are independent chi-square draws.
        checks['nominal_nees_mean_calibrated'] = (
            nees.get('mean') is not None and 0.5 <= nees['mean'] <= 6.0)
        checks['nominal_nees_95pct_coverage'] = .80 <= coverage <= .995
        for key in ('x', 'y', 'yaw'):
            channel = state_nees.get(key, {})
            channel_mean = channel.get('mean')
            c = state_coverage.get(key)
            checks[f'{key}_nees_mean_calibrated'] = (
                channel.get('count', 0) >= max(10, int(seconds)) and
                channel_mean is not None and .20 <= channel_mean <= 5.0)
            checks[f'{key}_95pct_coverage'] = (
                c is not None and .75 <= c <= .995)
    result = {
        'passed': all(checks.values()),
        'mode': 'strict' if strict else 'portable',
        'consistency_gate_enabled': consistency_gate,
        'checks': checks,
        'thresholds': {'callback_p99_ms': 6. if strict else 25.,
                       'deadline_miss_fraction': max_miss,
                       'nominal_nees_mean_if_enabled': [.5, 6.0],
                       'nominal_coverage_if_enabled': [.80, .995],
                       'marginal_nees_mean_if_enabled': [.20, 5.0],
                       'marginal_coverage_if_enabled': [.75, .995],
                       'min_independent_trajectory_blocks': 5},
        'metrics': {
            'elapsed_seconds': elapsed,
            'odom_hz': r.get('odom_hz'),
            'solver_p99_ms': solver.get('p99'),
            'callback_p99_ms': callback.get('p99'),
            'callback_deadline_miss_fraction': r.get('callback_deadline_miss_fraction'),
            'worker_age_solves': worker.get('age_solves'),
            'nees_nominal': cov.get('nominal_nees'),
            'nees_95pct_coverage_nominal': cov.get('nominal_95pct_coverage'),
            'state_nees_nominal': cov.get('nominal_state_nees'),
            'state_coverage_nominal': cov.get('nominal_state_95pct_coverage'),
            'phase_nees': cov.get('phase_nees'),
            'cycle_block_bootstrap': cov.get('independent_cycle_bootstrap'),
            'timestamp_bracket_ms': cov.get('alignment_bracket_ms'),
            'approximate_nis': nis.get('approximate_prefit_nis'),
        },
        'interpretation': 'CI synthetic-reference test only; not proof of calibrated physical-robot covariance.'}
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    r = json.loads(args.input.read_text(encoding='utf-8'))
    strict = os.environ.get('MHE_STRESS_STRICT', '0') == '1'
    consistency_gate = os.environ.get('MHE_STRESS_CONSISTENCY_GATE', '0') == '1'
    verdict = check_report(r, strict=strict, consistency_gate=consistency_gate)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(verdict, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(verdict, indent=2))
    raise SystemExit(0 if verdict['passed'] else 1)


if __name__ == '__main__':
    main()
