#!/usr/bin/env python3
"""Synthetic 9D marginal prior information-preservation benchmark.

Numerical reference only. This is NOT a Ceres/Gazebo navigation benchmark.
"""
import json
import os
from pathlib import Path
import numpy as np

rng = np.random.default_rng(20261008)
max_h_error = 0.0
max_g_error = 0.0
trials = 0
rank_counts = {str(k): 0 for k in range(1, 10)}
legacy_null_curvature = []
for rank in range(1, 10):
    for seed in range(40):
        Q, _ = np.linalg.qr(rng.normal(size=(9, 9)))
        eigen = np.zeros(9)
        eigen[:rank] = 0.3 + 0.5*np.arange(rank)
        D = np.array([1.0, 2.0, 1.3, 4.0, 0.8, 0.2, 0.1, 0.7, 1.4])
        H = (D[:, None]*Q)@np.diag(eigen)@Q.T*D[None, :]
        g = H @ rng.normal(size=9)
        normalized_scale = 1.0/np.sqrt(np.maximum(np.abs(np.diag(H)), max(np.max(np.abs(np.diag(H)))*1e-12, 1e-16)))
        W = normalized_scale[:, None]*H*normalized_scale[None, :]
        lam, U = np.linalg.eigh((W + W.T)/2)
        good = lam > max(0., lam.max())*1e-9
        sqrt_info = np.zeros((9, 9)); offset = np.zeros(9)
        for k in np.where(good)[0]:
            root = np.sqrt(lam[k])
            sqrt_info[k, :] = root * U[:, k]/normalized_scale
            offset[k] = (U[:, k] @ (normalized_scale*g))/root
        assert int(good.sum()) == rank
        he = np.linalg.norm(H-sqrt_info.T@sqrt_info)/max(1., np.linalg.norm(H))
        ge = np.linalg.norm(g-sqrt_info.T@offset)/max(1., np.linalg.norm(g))
        max_h_error=max(max_h_error, float(he))
        max_g_error=max(max_g_error, float(ge))
        assert he < 1e-8 and ge < 1e-8
        if rank < 9:
            # Old implementation forces *all* zero eigenvalues >= 1e-7.
            evals, vec = np.linalg.eigh(H)
            legacy = (vec*np.maximum(evals,1e-7))@vec.T
            nullvec = Q[:, rank] / D
            nullvec /= np.linalg.norm(nullvec)
            legacy_null_curvature.append(float(nullvec@legacy@nullvec))
        rank_counts[str(rank)]+=1
        trials += 1

summary = dict(type='synthetic_numpy_reference_not_ros', seed=20261008,
               trials=trials, ranks=rank_counts,
               max_relative_information_error=max_h_error,
               max_relative_gradient_error=max_g_error,
               median_spurious_legacy_null_curvature=float(np.median(legacy_null_curvature)),
               pass_all=True)
report_name = 'benchmark_rank_aware_prior.json'
report_text = json.dumps(summary, indent=2) + '\n'
# Regression gates and CI harness always read the canonical benchmark directory.
(Path(__file__).parent / report_name).write_text(report_text, encoding='utf-8')
# Preserve exported reports for GitHub Actions artifacts when configured.
if os.environ.get('CI_ARTIFACT_DIR'):
    artifacts = Path(os.environ['CI_ARTIFACT_DIR'])
    artifacts.mkdir(parents=True, exist_ok=True)
    (artifacts / report_name).write_text(report_text, encoding='utf-8')
print(json.dumps(summary, indent=2))
