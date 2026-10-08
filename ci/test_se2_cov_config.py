"""Configuration and source wiring invariants for A/B mathematical features."""
import unittest
from pathlib import Path
import yaml
ROOT = Path(__file__).resolve().parents[1]

class TestMathConfig(unittest.TestCase):
    def params(self, name):
        return yaml.safe_load((ROOT / 'config' / name).read_text())['mhe_sensor_fusion']['ros__parameters']

    def test_baseline_safe(self):
        for name in ('mhe.yaml', 'mhe_user_baseline.yaml', 'mhe_gazebo_low_latency.yaml'):
            p = self.params(name)
            self.assertFalse(p.get('covariance', {}).get('jacobi_scaled_enabled', False))
            self.assertFalse(p.get('output', {}).get('se2_correction', {}).get('enabled', False))

    def test_experiment_optin(self):
        p = self.params('mhe_gazebo_math_ab.yaml')
        self.assertTrue(p['use_sim_time'])
        self.assertTrue(p['covariance']['jacobi_scaled_enabled'])
        self.assertTrue(p['output']['se2_correction']['enabled'])
        self.assertFalse(p['output']['yaw_feedback']['enabled'])
        self.assertFalse(p['output']['smoothing']['angular_enabled'])
        self.assertEqual(p['transform_time_offset'], 0.0)

    def test_in_node(self):
        src = (ROOT / 'src/mhe_sensor_fusion.cpp').read_text()
        for marker in ('se2_correct::reconcile(', 'se2_correct::predict(',
                       'covariance_jacobi_scaled_enabled_',
                       'covariance_minimum_rcond_', 'se2_correction_policy_'):
            self.assertIn(marker, src)
        self.assertIn('se2_correction_policy_.enabled = false', src)

    def test_worker_reports_failure_honestly(self):
        header = (ROOT / 'include/mhe_sensor_fusion/scaled_covariance.hpp').read_text()
        self.assertIn('if (output.rank != cols', header)
        self.assertIn('maximum_svd_condition', header)
        self.assertIn('Eigen::JacobiSVD', header)

if __name__ == '__main__':
    unittest.main()
