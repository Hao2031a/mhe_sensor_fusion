"""Guard parameter namespace and A/B defaults for analytic factor rollout."""
import unittest
from pathlib import Path
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class AnalyticConfigTest(unittest.TestCase):
    def test_solver_parameter_is_nested(self):
        for filename in ('mhe.yaml', 'mhe_gazebo_low_latency.yaml', 'mhe_user_baseline.yaml'):
            with self.subTest(filename=filename):
                params = yaml.safe_load((PACKAGE / 'config' / filename).read_text())['mhe_sensor_fusion']['ros__parameters']
                self.assertNotIn('analytic_factors_enabled', params)
                self.assertIn('analytic_factors_enabled', params['solver'])
                expected = filename != 'mhe_user_baseline.yaml'
                self.assertIs(params['solver']['analytic_factors_enabled'], expected)

    def test_source_uses_ceres_either_way(self):
        src = (PACKAGE / 'src/mhe_sensor_fusion.cpp').read_text()
        self.assertIn('new AnalyticProcessCost', src)
        self.assertIn('new AnalyticWheelPairCost', src)
        self.assertIn('new ceres::AutoDiffCostFunction<ProcessCost', src)
        self.assertIn('new ceres::AutoDiffCostFunction<WheelPairCost', src)


if __name__ == '__main__':
    unittest.main()
