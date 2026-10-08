#!/usr/bin/env python3
import math
import unittest
import numpy as np
from consistency_metrics import (BoundedSamples, PoseTruthBuffer, nees_pose,
                                 pose_covariance_3x3, wrap)


class ConsistencyTests(unittest.TestCase):
    def test_nees_exact(self):
        cov = np.eye(6)
        value, eig = nees_pose([1., 1., 1.], [0., 0., 0.], cov.flatten())
        self.assertAlmostEqual(value, 3.)
        self.assertAlmostEqual(eig, 1.)

    def test_covariance_extraction_offdiagonal(self):
        p = np.eye(6)
        p[0, 5] = p[5, 0] = .25
        self.assertAlmostEqual(pose_covariance_3x3(p.flatten())[0, 2], .25)

    def test_invalid_covariance_rejected(self):
        cov = np.eye(6)
        cov[0, 0] = -.1
        nees, eig = nees_pose([0., 0., 0.], [0., 0., 0.], cov.flatten())
        self.assertIsNone(nees)
        self.assertLess(eig, 0)

    def test_singular_covariance_rejected(self):
        cov = np.eye(6)
        cov[5, 5] = 0.0
        self.assertIsNone(nees_pose([0., 0., 0.], [0., 0., 0.], cov.flatten())[0])

    def test_yaw_wrap(self):
        self.assertLess(abs(wrap(-math.pi + .01 - (math.pi - .01))), .021)

    def test_interpolated_truth(self):
        truth = PoseTruthBuffer()
        truth.add(1_000_000_000, 0., 0., 0.)
        truth.add(1_010_000_000, 1., 0., .2)
        x = truth.lookup(1_005_000_000)
        self.assertAlmostEqual(x[0], .5)
        self.assertAlmostEqual(x[2], .1)
        self.assertTrue(x[3])

    def test_invalid_phase_truth(self):
        truth = PoseTruthBuffer()
        truth.add(1_000_000_000, 0., 0., 0., True)
        truth.add(1_010_000_000, 1., 0., 0., False)
        self.assertFalse(truth.lookup(1_005_000_000)[3])

    def test_stats_bounded(self):
        stats = BoundedSamples(stride=1, max_samples=100)
        for i in range(1000):
            stats.add(float(i))
        self.assertEqual(stats.count, 1000)
        self.assertLessEqual(len(stats.samples), 100)
        self.assertGreater(stats.stats()['p99'], stats.stats()['p50'])


if __name__ == '__main__':
    unittest.main()
