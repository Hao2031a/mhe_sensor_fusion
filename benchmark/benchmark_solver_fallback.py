#!/usr/bin/env python3
"""Deterministic *policy simulation*, not an actual Ceres/ROS timing benchmark.

Exercises a matched 30-seed event workload through the old (full-budget retry
on any rejected solution) versus new (reason + remaining-budget gate) policies.
This diagnoses unnecessary retry *eligibility* and nominal budget scheduling;
it cannot establish achieved runtime or localization accuracy.
"""
import json
import random
import statistics
from pathlib import Path

SEEDS = 30
EVENTS = 1000
BUDGET_MS = 6.0
MIN_REMAINING_MS = 1.0


def trial(seed):
    rng = random.Random(seed)
    old_retries = new_retries = numeric_lost = 0
    old_planned = new_planned = 0.0
    endpoint_rejections = 0
    numerical_rejections = 0
    cost_rejections = 0
    for _ in range(EVENTS):
        r = rng.random()
        # Synthetic rejection reasons; probabilities are a test fixture, not
        # measurements from Gazebo or real ROS node.
        reason = ('numerical' if r < 0.008 else
                  'cost' if r < 0.018 else
                  'endpoint' if r < 0.05 else 'none')
        if reason == 'endpoint':
            endpoint_rejections += 1
        if reason == 'numerical':
            numerical_rejections += 1
        if reason == 'cost':
            cost_rejections += 1
        # Typical solve under 6ms; rare expensive solve at/over budget.
        first_ms = rng.uniform(0.8, 3.3) if rng.random() < 0.97 else rng.uniform(5.4, 7.2)
        old_retry = reason != 'none'
        if old_retry:
            old_retries += 1
        # Old fallback re-issued max_solver_time_in_seconds = full budget.
        old_planned += first_ms + (BUDGET_MS if old_retry else 0.0)
        remaining = max(0, BUDGET_MS - first_ms)
        new_retry = reason in ('numerical', 'cost') and remaining >= MIN_REMAINING_MS
        if new_retry:
            new_retries += 1
        elif reason in ('numerical', 'cost'):
            numeric_lost += 1
        new_planned += first_ms + (remaining if new_retry else 0.0)
    return dict(old_retries=old_retries, new_retries=new_retries,
                old_planned_ms=old_planned, new_planned_ms=new_planned,
                endpoint_rejections=endpoint_rejections,
                numerical_rejections=numerical_rejections,
                cost_rejections=cost_rejections,
                skipped_numeric_retries_low_budget=numeric_lost)


def main():
    results = [trial(i) for i in range(SEEDS)]
    means = {k: statistics.mean(row[k] for row in results) for k in results[0]}
    means['retry_reduction_pct'] = 100 * (1-means['new_retries']/means['old_retries'])
    means['planned_solver_budget_reduction_pct'] = 100 * (
        1-means['new_planned_ms']/means['old_planned_ms'])
    checks = {
        'new_retries_less_than_old': means['new_retries'] < means['old_retries'],
        'new_planned_budget_lower': means['new_planned_ms'] < means['old_planned_ms'],
        'redundant_retries_reduced_gt_50pct': means['retry_reduction_pct'] > 50,
        'nonzero_numerical_retry_coverage': means['new_retries'] > 0,
    }
    report = {'synthetic_only': True, 'gazebo_tested': False,
              'real_ceres_timed': False, 'trials': SEEDS,
              'events_per_trial': EVENTS, 'budget_ms': BUDGET_MS,
              'metrics_mean_per_trial': means, 'checks': checks,
              'pass': all(checks.values())}
    (Path(__file__).with_name('benchmark_solver_fallback.json')).write_text(
        json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))
    if not report['pass']:
        raise SystemExit('FAIL: synthetic solver fallback policy benchmark')


if __name__ == '__main__':
    main()
