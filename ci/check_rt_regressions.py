#!/usr/bin/env python3
"""Real ROS/Ceres smoke performance regression gate.

Timing on shared CI runners is not hardware WCET. Strict 100-Hz thresholds are
opt-in (MHE_RT_STRICT=1) for a fixed, self-hosted performance runner.
"""
import argparse
import json
import os
from pathlib import Path


def check_report(report, *, strict=False, p99_limit_ms=None):
    rt = report.get('realtime_profile', {})
    solver = rt.get('total_solve_ms', {})
    ceres = rt.get('ceres_only_ms', {})
    callback = rt.get('callback_wall_ms', {})
    worker_ready = rt.get('covariance_worker_completed', 0) >= 2
    worker_valid = rt.get('covariance_worker_valid', 0) == 1.0
    p99_limit = float(p99_limit_ms if p99_limit_ms is not None else (10.0 if strict else 25.0))
    checks = {
        'ros_smoke_functionality': bool(report.get('pass')),
        'cov_worker_completes': worker_ready,
        'cov_worker_valid': worker_valid,
        'cov_worker_no_failures': rt.get('covariance_worker_failed', 1) == 0,
        'rt_profile_collected': solver.get('count', 0) >= 20 and callback.get('count', 0) >= 80,
        'solver_p99_under_limit': solver.get('p99') is not None and solver['p99'] <= p99_limit,
        'ceres_p99_bounded': ceres.get('p99') is not None and ceres['p99'] <= (10.0 if strict else 20.0),
        'rollback_count_zero': rt.get('rollback_count', 1) == 0,
        'deadline_fraction_bounded': rt.get('solve_deadline_miss_fraction', 1.0) <=
        (0.01 if strict else 0.35),
        'callback_overrun_fraction_bounded': rt.get('callback_deadline_miss_fraction', 1.0) <=
        (0.01 if strict else 0.35),
    }
    if strict:
        checks['solver_p95_below_8ms'] = solver.get('p95') is not None and solver['p95'] < 8.0
        checks['solver_p99_below_5ms'] = solver.get('p99') is not None and solver['p99'] < 5.0
        checks['callback_p99_below_6ms'] = callback.get('p99') is not None and callback['p99'] < 6.0
        checks['callback_miss_under_0_1pct'] = rt.get('callback_deadline_miss_fraction', 1.0) < .001
        checks['cov_worker_age_under_120_solves'] = rt.get('covariance_worker_age_solves') is not None and rt['covariance_worker_age_solves'] < 120
        checks['cov_worker_failures_zero'] = rt.get('covariance_worker_failed', 1) == 0
    return {'passed': all(checks.values()), 'mode': 'strict' if strict else 'portable',
            'checks': checks, 'thresholds': {'p99_ms': p99_limit,
            'deadline_miss_fraction': .01 if strict else .35},
            'metrics': {'total_solve_ms': solver, 'ceres_only_ms': ceres,
            'callback_wall_ms': callback,
            'cov_worker_completed': rt.get('covariance_worker_completed'),
            'cov_worker_compute_ms': rt.get('covariance_worker_compute_ms'),
            'cov_worker_age_solves': rt.get('covariance_worker_age_solves'),
            'solve_deadline_miss_fraction': rt.get('solve_deadline_miss_fraction'),
            'callback_deadline_miss_fraction': rt.get('callback_deadline_miss_fraction')}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', type=Path, required=True)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    report = json.loads(args.input.read_text(encoding='utf-8'))
    strict = os.getenv('MHE_RT_STRICT', '0') == '1'
    value = os.getenv('MHE_RT_P99_MAX_MS')
    result = check_report(report, strict=strict,
                          p99_limit_ms=float(value) if value else None)
    print(json.dumps(result, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
