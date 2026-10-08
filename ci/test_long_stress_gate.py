#!/usr/bin/env python3
import copy
import unittest
from check_long_stress import check_report


def good_report():
    return {
        'kind': 'ros_node_synthetic_ground_truth_stress',
        'configured_seconds': 30, 'elapsed_seconds': 30.3,
        'odom_count': 2990, 'odom_nonfinite_count': 0,
        'odom_nonmonotonic_stamp_count': 0, 'max_odom_gap_ms': 20,
        'solver_ms': {'count': 2500, 'p99': 1.5},
        'callback_ms': {'count': 3000, 'p99': 2.1},
        'callback_deadline_miss_fraction': 0,
        'solver_deadline_miss_fraction': 0, 'solver_rollbacks': 0,
        'covariance_worker': {'completed': 150, 'failed': 0, 'valid': 1, 'age_solves': 10},
        'covariance_consistency': {'invalid_or_negative_eigen_count': 0,
            'pose_error_nees': {'count': 260},
            'nominal_nees_sample_count': 150,
            'nominal_nees': {'mean': 3}, 'nominal_95pct_coverage': .95,
            'nominal_state_nees': {k: {'count': 150, 'mean': 1.} for k in ('x','y','yaw')},
            'nominal_state_95pct_coverage': {k: .95 for k in ('x','y','yaw')},
            'independent_cycle_bootstrap': {'valid': True, 'independent_blocks': 5},
            'alignment_bracket_ms': {'count': 200, 'p99': 10.},
            'alignment_extrapolation_allowed': False},
        'innovation_consistency': {'approximate_prefit_nis': [{'count': 200}]*4},
        'injections': {'encoder_spike': 1, 'imu_dropout_ticks': 200,
                       'wheel_dropout_ticks': 100, 'imu_out_of_order': 1}}


class GateTest(unittest.TestCase):
    def test_pass(self):
        self.assertTrue(check_report(good_report())['passed'])
        self.assertTrue(check_report(good_report(), strict=True)['passed'])
        self.assertTrue(check_report(good_report(), consistency_gate=True)['passed'])

    def test_each_fault_causes_fail(self):
        updates = {
            'odom_nonfinite_count': 1,
            'odom_nonmonotonic_stamp_count': 1,
            'callback_deadline_miss_fraction': .11,
            'solver_rollbacks': 100,
            'max_odom_gap_ms': 1000,
        }
        for key, val in updates.items():
            r = good_report()
            r[key] = val
            with self.subTest(key=key):
                self.assertFalse(check_report(r)['passed'])

    def test_invalid_covariance_fails(self):
        r = good_report()
        r['covariance_consistency']['invalid_or_negative_eigen_count'] = 4
        self.assertFalse(check_report(r)['passed'])

    def test_inconsistency_fails_when_enabled(self):
        r = good_report()
        r['covariance_consistency']['nominal_95pct_coverage'] = .05
        self.assertTrue(check_report(r)['passed'])
        self.assertFalse(check_report(r, consistency_gate=True)['passed'])

    def test_strict_overrun_detected(self):
        r = good_report()
        r['callback_ms']['p99'] = 8
        self.assertTrue(check_report(r)['passed'])
        self.assertFalse(check_report(r, strict=True)['passed'])

    def test_short_run_does_not_require_out_of_order(self):
        r = good_report()
        r['configured_seconds'] = 15
        r['injections'] = {}
        self.assertTrue(check_report(r)['passed'])

    def test_missing_timestamp_alignment_blocks_consistency(self):
        r = good_report()
        del r['covariance_consistency']['alignment_bracket_ms']
        self.assertFalse(check_report(r, consistency_gate=True)['passed'])

    def test_missing_statewise_consistency_fails(self):
        r = good_report()
        del r['covariance_consistency']['nominal_state_nees']
        self.assertFalse(check_report(r, consistency_gate=True)['passed'])

    def test_overconservative_covariance_fails(self):
        r = good_report()
        r['covariance_consistency']['nominal_95pct_coverage'] = 1.0
        self.assertFalse(check_report(r, strict=True, consistency_gate=True)['passed'])

    def test_missing_nis_fails(self):
        r = good_report()
        r['innovation_consistency']['approximate_prefit_nis'] = []
        self.assertFalse(check_report(r)['passed'])


if __name__ == '__main__':
    unittest.main()
