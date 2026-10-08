import math
import unittest

from yaw_metrics import evaluate, interpolate_yaw


class YawMetricsTest(unittest.TestCase):
    def test_wrap_interpolation(self):
        samples = [(0.0, math.radians(179)), (0.1, math.radians(-179))]
        got = interpolate_yaw(samples, 0.05)
        self.assertAlmostEqual(abs(got), math.pi, places=4)

    def test_relative_offset_cancels(self):
        truth = [(i * .02, 0.1 * i * .02) for i in range(100)]
        est = [(i * .01, 0.1 * i * .01 + .25) for i in range(198)]
        result = evaluate(truth, est)
        self.assertLess(result['relative_yaw_rmse_rad'], 1e-10)

    def test_detects_drift(self):
        truth = [(i * .02, 0.0) for i in range(100)]
        est = [(i * .01, 0.10 * i * .01) for i in range(198)]
        result = evaluate(truth, est)
        self.assertGreater(result['relative_yaw_rmse_rad'], .10)

    def test_reject_missing(self):
        with self.assertRaises(ValueError):
            evaluate([(0.0, 0.0)], [(i * .01, 0.0) for i in range(20)])

    def test_reject_gaps(self):
        self.assertIsNone(interpolate_yaw([(0, 0), (1, 0)], 0.5))

if __name__ == '__main__':
    unittest.main()
