#!/usr/bin/env python3
"""Deterministic correctness/regression gates. Do not compare wall-clock runtime
across shared CI runners; record timing from the ROS test separately."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--package', type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    d = args.package / 'benchmark'
    load = lambda name: json.loads((d / name).read_text(encoding='utf-8'))
    upgrade = load('benchmark_upgrade.json')
    stability = load('benchmark_stability.json')
    mc = load('benchmark_stability_mc.json')
    gate = load('benchmark_innovation_gate.json')
    consistency = load('benchmark_consistency_mc.json')
    yaw = load('benchmark_yaw_latency.json')
    qr = load('benchmark_solver_fallback.json')
    aligned = load('benchmark_timestamp_yaw.json')
    schur = load('benchmark_block_schur.json')
    rank_prior = load('benchmark_rank_aware_prior.json')
    yaw_so2 = load('benchmark_so2_yaw.json')
    gyro_increment = load('benchmark_gyro_increment.json')
    checks = {
        'gyro_increment_math_synthetic_noise_envelope': bool(gyro_increment['pass']) and len(gyro_increment['results']) == 12 and min(s['within_95pct_noise_envelope'] for s in gyro_increment['results']) >= .94,
        'so2_yaw_and_exact_se2_math': bool(yaw_so2['pass']) and
            yaw_so2['max_exact_arc_rmse_m'] < 2e-8 and
            yaw_so2['max_exact_to_midpoint_error_ratio_high_turn'] < 0.005,
        'deterministic_output': bool(stability['overall_pass']),
        'yaw_latency_synthetic': bool(yaw['pass']),
        'qr_fallback_policy_synthetic': bool(qr['pass']),
        'timestamp_yaw_policy_synthetic': bool(aligned['pass']),
        'block_schur_matches_dense_synthetic': bool(schur['pass']) and schur['max_abs_h_error'] < 1e-8 and schur['max_abs_rhs_error'] < 1e-8,
        'rank_aware_sqrt_prior_preserves_information': bool(rank_prior['pass_all']) and rank_prior['trials'] >= 360 and rank_prior['max_relative_information_error'] < 1e-8 and rank_prior['max_relative_gradient_error'] < 1e-8,
        'monte_carlo_output': bool(mc['overall_pass']),
        'innovation_gate_and_covariance': bool(gate['pass']),
        'nees_nis_independent_calibration': bool(consistency['pass']),
        'linear_jitter_improvement_ge_15_percent': stability['linear_output']['jitter_reduction_vs_baseline_percent'] >= 15,
        'legacy_angular_shaper_jitter_improvement_ge_15_percent': stability['angular_output']['jitter_reduction_vs_baseline_percent'] >= 15,
        'common_slip_peak_bounded': abs(upgrade['common_slip']['peak_estimate']) <= 0.35,
        'common_slip_excitation_learns': upgrade['common_slip']['estimate_at_end_excitation'] >= 0.10,
        'accel_corrected_rmse_below_0_25': upgrade['accelerometer_robustness']['rmse_new_mps2'] < 0.25,
        'covariance_n20_relative_error_below_0_05': upgrade['covariance_consistency']['20']['hessian_relative_error'] < 0.05,
    }
    result = {'passed': all(checks.values()), 'checks': checks,
              'metrics': {'yaw_rmse_improvement_synthetic_pct': yaw['metrics']['rmse_improvement_percent'],
                          'qr_retry_reduction_synthetic_pct': qr['metrics_mean_per_trial']['retry_reduction_pct'],
                          'linear_jitter_reduction_pct': stability['linear_output']['jitter_reduction_vs_baseline_percent'],
                          'angular_jitter_reduction_pct': stability['angular_output']['jitter_reduction_vs_baseline_percent'],
                          'common_slip_peak': upgrade['common_slip']['peak_estimate'],
                          'accel_rmse_mps2': upgrade['accelerometer_robustness']['rmse_new_mps2']}}
    print(json.dumps(result, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
