#!/usr/bin/env python3
import json
import math
import tempfile
import unittest
from pathlib import Path
import numpy as np
from consistency_metrics import (PoseTruthBuffer, cycle_block_bootstrap,
                                 nees_statewise, nees_pose)
from propose_covariance_calibration import propose


class StatisticalCalibrationTests(unittest.TestCase):
    def test_strict_time_alignment_no_extrapolation(self):
        ref=PoseTruthBuffer()
        ref.add(1_000_000_000,0,0,0,phase='straight',cycle=1)
        ref.add(1_010_000_000,1,1,.1,phase='straight',cycle=1)
        self.assertIsNone(ref.lookup_aligned(1_011_000_000))
        self.assertIsNone(ref.lookup_aligned(999_000_000))
        m=ref.lookup_aligned(1_005_000_000)
        self.assertAlmostEqual(m['pose'][0],.5)
        self.assertAlmostEqual(m['bracket_ms'],10.)
        self.assertTrue(m['valid'])
        self.assertEqual(m['phase'],'straight')

    def test_phase_transition_excluded(self):
        ref=PoseTruthBuffer()
        ref.add(1_000_000_000,0,0,3.13,phase='turn',cycle=1)
        ref.add(1_010_000_000,0,0,-3.13,phase='slip',cycle=1)
        m=ref.lookup_aligned(1_005_000_000)
        self.assertFalse(m['valid'])
        self.assertEqual(m['phase'],'transition')
        self.assertAlmostEqual(abs(m['pose'][2]),math.pi,delta=.02)

    def test_wide_timestamp_gap_rejected(self):
        ref=PoseTruthBuffer()
        ref.add(100_000_000,0,0,0)
        ref.add(300_000_000,1,0,0)
        self.assertIsNone(ref.lookup_aligned(200_000_000))

    def test_marginal_nees(self):
        p=np.eye(6)
        p[0,0]=4.;p[1,1]=9.;p[5,5]=.25
        z=nees_statewise((2,3,.5),(0,0,0),p.flatten())
        self.assertTrue(np.allclose(z,(1,1,1)))
        self.assertAlmostEqual(nees_pose((2,3,.5),(0,0,0),p.flatten())[0],3.)

    def test_bootstrap_requires_separate_cycles(self):
        self.assertFalse(cycle_block_bootstrap({0:{'n':300,'sum_nees':900,'covered':285}})['valid'])
        b={i:{'n':100,'sum_nees':300.,'covered':95} for i in range(10)}
        v=cycle_block_bootstrap(b,replicates=100)
        self.assertTrue(v['valid'])
        self.assertEqual(v['independent_blocks'],10)
        self.assertTrue(np.allclose(v['mean_nees_ci95'],[3.,3.]))
        self.assertTrue(np.allclose(v['coverage_ci95'],[.95,.95]))

    def test_holdout_fit_detects_conservative_covariance(self):
        with tempfile.TemporaryDirectory() as d:
            paths=[]
            for seed in (11,22,33):
                rng=np.random.default_rng(seed)
                records=[]
                for j in range(1100):
                    truth_err=rng.normal(0.,[.02,.02,.03])
                    p=np.diag(np.square([.04,.04,.06]))
                    records.append({'nominal':True,'cycle':j//110,
                                    'error':truth_err.tolist(),'P':p.tolist()})
                path=Path(d)/f'{seed}.json'
                path.write_text(json.dumps({'seed':seed,'synthetic_ground_truth_only':True,
                    'covariance_consistency':{'aligned_pose_records':records}}))
                paths.append(path)
            output=propose(paths[:2],paths[2])
            self.assertTrue(output['candidate_only'])
            self.assertFalse(output['automatically_applied_to_mhe'])
            self.assertTrue(all(.5 <= v < 1 for v in output['candidate_pose_std_scales']))
            self.assertTrue(output['holdout_improves_consistency_distance'])
            self.assertGreater(output['holdout_candidate']['nees_mean'],output['holdout_baseline']['nees_mean'])

    def test_holdout_rejects_seed_reuse(self):
        with self.assertRaises((ValueError,FileNotFoundError)):
            propose(['missing.json','missing.json'],'missing.json')

if __name__=='__main__':
    unittest.main()
