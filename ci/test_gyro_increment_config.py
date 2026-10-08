from pathlib import Path
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
class TestGyroIncrementConfig(unittest.TestCase):
 def test_node_has_safe_default(self):
  src=(ROOT/'src/mhe_sensor_fusion.cpp').read_text()
  self.assertIn('"solver.gyro_increment_factor_enabled", false',src)
  self.assertIn('z.has_gyro && !useGyroIncrement(k)',src)
  self.assertIn('if (useGyroIncrement(k))',src)
  self.assertIn('new AnalyticGyroIncrementCost',src)
  self.assertIn('gyro_inc_cost, new ceres::HuberLoss(1.5)',src)
 def test_independent_optin(self):
  for name, expected in (('mhe.yaml',False),('mhe_user_baseline.yaml',False),('mhe_gazebo_low_latency.yaml',True)):
   raw=yaml.safe_load((ROOT/'config'/name).read_text())
   s=raw['mhe_sensor_fusion']['ros__parameters']['solver']
   self.assertIs(s.get('gyro_increment_factor_enabled',False),expected)
  opt=yaml.safe_load((ROOT/'config/mhe_gazebo_low_latency.yaml').read_text())['mhe_sensor_fusion']['ros__parameters']
  self.assertFalse(opt['output']['smoothing']['angular_enabled'])
  self.assertTrue(opt['solver']['so2_yaw_residual_enabled'])
 def test_only_local_chain_edges(self):
  src=(ROOT/'src/mhe_sensor_fusion.cpp').read_text()
  self.assertIn('states_[k - 1].data(), states_[k].data()',src)
  self.assertIn('dt <= gyro_increment_max_dt_',src)
if __name__=='__main__':unittest.main()
