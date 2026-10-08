"""Safety regression: experimental mathematical mode must remain opt-in."""
import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


class RankAwareConfigTest(unittest.TestCase):
    def get_solver(self, name):
        content = yaml.safe_load((ROOT / 'config' / name).read_text())
        return content['mhe_sensor_fusion']['ros__parameters']['solver']

    def test_default_config_is_not_silently_modified(self):
        self.assertIs(self.get_solver('mhe.yaml')['rank_aware_prior_enabled'], False)
        self.assertIs(self.get_solver('mhe_user_baseline.yaml')['rank_aware_prior_enabled'], False)

    def test_gazebo_experiment_can_be_reverted(self):
        solver = self.get_solver('mhe_gazebo_low_latency.yaml')
        self.assertIs(solver['rank_aware_prior_enabled'], True)
        self.assertTrue(0 < solver['prior_eigen_relative_cutoff'] < 1e-6)
        self.assertTrue(0 < solver['prior_max_discarded_gradient_fraction'] < 1e-2)
        self.assertIs(solver['incremental_graph_enabled'], True)
        self.assertIs(solver['block_schur_enabled'], True)
        params = yaml.safe_load((ROOT/'config/mhe_gazebo_low_latency.yaml').read_text())['mhe_sensor_fusion']['ros__parameters']
        self.assertIs(params['output']['smoothing']['angular_enabled'], False)
        self.assertIs(params['timing']['timestamp_aligned_output_enabled'], True)


if __name__ == '__main__':
    unittest.main()
