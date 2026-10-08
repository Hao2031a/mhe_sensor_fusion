#!/usr/bin/env python3
"""Check that timing regressions return failure even with optimized Python."""
from check_rt_regressions import check_report


def require(condition, detail):
    if not condition:
        raise RuntimeError('Timing regression gate test failed: ' + detail)


sample = {'pass': True, 'realtime_profile': {
    'total_solve_ms': {'count': 100, 'p95': 3.0, 'p99': 4.5, 'max': 11.0},
    'ceres_only_ms': {'count': 100, 'p95': 4.0, 'p99': 6.0, 'max': 7.0},
    'callback_wall_ms': {'count': 100, 'p95': 7.0, 'p99': 5.5, 'max': 11.0},
    'solve_deadline_miss_fraction': 0.0,
    'callback_deadline_miss_fraction': 0.0,
    'rollback_count': 0,
    'covariance_worker_completed': 10,
    'covariance_worker_valid': 1.0,
    'covariance_worker_age_solves': 20,
    'covariance_worker_failed': 0,
}}
require(check_report(sample)['passed'], 'positive portable')
require(check_report(sample, strict=True)['passed'], 'positive strict')
sample['realtime_profile']['covariance_worker_completed'] = 0
require(not check_report(sample)['passed'], 'missing worker results rejection')
sample['realtime_profile']['covariance_worker_completed'] = 10
sample['realtime_profile']['total_solve_ms']['p99'] = 26.0
require(not check_report(sample)['passed'], 'high solver p99 rejection')
sample['realtime_profile']['total_solve_ms']['p99'] = 8.0
sample['realtime_profile']['callback_deadline_miss_fraction'] = 0.4
require(not check_report(sample)['passed'], 'callback misses rejection')
sample['realtime_profile']['callback_deadline_miss_fraction'] = 0.0
sample['realtime_profile']['rollback_count'] = 1
require(not check_report(sample)['passed'], 'rollback rejection')
sample['realtime_profile']['rollback_count'] = 0
sample['realtime_profile']['covariance_worker_age_solves'] = 121
require(not check_report(sample, strict=True)['passed'], 'strict covariance staleness rejection')
sample['realtime_profile']['covariance_worker_age_solves'] = 20
sample['realtime_profile']['covariance_worker_failed'] = 1
require(not check_report(sample, strict=True)['passed'], 'strict worker failure rejection')
sample['realtime_profile']['covariance_worker_failed'] = 0
sample['realtime_profile']['callback_wall_ms']['p99'] = 6.5
require(not check_report(sample, strict=True)['passed'], 'strict callback P99 rejection')
print('PASS: positive, strict, worker, slow-solver, callback-overrun and rollback regression controls')
