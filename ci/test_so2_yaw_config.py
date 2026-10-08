"""SO2 yaw / SE2 arc choices: Gazebo experimental, legacy untouched."""
from pathlib import Path
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
class TestYawConfiguration(unittest.TestCase):
    def solver(self,name):
        raw=yaml.safe_load((ROOT/'config'/name).read_text())
        return raw['mhe_sensor_fusion']['ros__parameters']['solver']
    def test_legacy_no_silent_enable(self):
        for name in ('mhe.yaml','mhe_user_baseline.yaml'):
            self.assertIsNot(self.solver(name).get('so2_yaw_residual_enabled',False),True)
            self.assertIsNot(self.solver(name).get('exact_se2_motion_enabled',False),True)
    def test_gazebo_only_optin(self):
        s=self.solver('mhe_gazebo_low_latency.yaml')
        self.assertIs(s['so2_yaw_residual_enabled'],True)
        self.assertIs(s['exact_se2_motion_enabled'],True)
        self.assertIs(s['analytic_factors_enabled'],True)
        conf=yaml.safe_load((ROOT/'config/mhe_gazebo_low_latency.yaml').read_text())
        p=conf['mhe_sensor_fusion']['ros__parameters']
        self.assertIs(p['use_sim_time'],True)
        self.assertIs(p['output']['smoothing']['angular_enabled'],False)
        self.assertEqual(p['transform_time_offset'],0.0)
    def test_source_wires_both_analytic_and_autodiff(self):
        src=(ROOT/'src/mhe_sensor_fusion.cpp').read_text()
        self.assertIn('new AnalyticProcessCost(dt, process_sigma,',src)
        self.assertIn('new ProcessCost(dt, process_sigma,',src)
        self.assertIn('arcWithDerivatives(pub_yaw_',src)
        self.assertIn('so2_yaw_residual_enabled_',src)
if __name__=='__main__': unittest.main()
