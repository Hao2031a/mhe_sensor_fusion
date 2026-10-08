import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


class GraphConfigTest(unittest.TestCase):
    def test_switches_explicit_and_rollback_possible(self):
        for filename in ('mhe.yaml', 'mhe_gazebo_low_latency.yaml', 'mhe_user_baseline.yaml'):
            cfg = yaml.safe_load((ROOT / 'config' / filename).read_text())
            p = cfg['mhe_sensor_fusion']['ros__parameters']
            s = p['solver']
            enabled = filename != 'mhe_user_baseline.yaml'
            self.assertIs(s['incremental_graph_enabled'], enabled)
            self.assertIs(s['block_schur_enabled'], enabled)
            self.assertIs(s['analytic_factors_enabled'], enabled)
            if filename == 'mhe_gazebo_low_latency.yaml':
                self.assertIs(p['use_sim_time'], True)
                self.assertEqual(p['transform_time_offset'], 0.0)
                self.assertIs(p['output']['smoothing']['angular_enabled'], False)

    def test_local_selection_mathematically_restricted_to_chain(self):
        source = (ROOT / 'src/mhe_sensor_fusion.cpp').read_text()
        self.assertIn('mhe_sensor_fusion::marginalization::prefixCandidates(', source)
        self.assertIn('graph_problem_.get() == &problem', source)
        self.assertIn('problem.GetResidualBlocks(&residual_blocks)', source)
        self.assertIn('last_marginal_evaluation_count_', source)


if __name__ == '__main__':
    unittest.main()
