#!/usr/bin/env python3
"""Independent Gaussian NEES/NIS calibration checks of the metric definitions.

Not a claim about the real MHE's covariance calibration. This check makes sure
that the consistency benchmark distinguishes correct and overconfident P/R.
"""
import json
from pathlib import Path
import numpy as np

N = 8000
rng = np.random.default_rng(20261008)
# Independent trials from a 3-D correlated covariance and 1-D innovation.
P = np.array([[.012, .003, -.002], [.003, .018, .001], [-.002, .001, .008]])
truth_error = rng.multivariate_normal(np.zeros(3), P, size=N)
empirical_nees = np.einsum('ni,ij,nj->n', truth_error, np.linalg.inv(P), truth_error)
overconfident_nees = np.einsum('ni,ij,nj->n', truth_error, np.linalg.inv(P*.20), truth_error)
innovation = rng.normal(0, .09, N)
nis = (innovation / .09) ** 2
bad_nis = (innovation / .03) ** 2
coverage = float(np.mean(empirical_nees < 7.814727903))
nis_coverage = float(np.mean(nis < 3.841458821))
correct_mean = float(np.mean(empirical_nees))
wrong_mean = float(np.mean(overconfident_nees))
result = {
    'kind': 'independent_gaussian_metric_calibration_only', 'samples': N,
    'nees_mean_correct_covariance': correct_mean,
    'nees_mean_overconfident_covariance': wrong_mean,
    'nees_95pct_coverage_correct_covariance': coverage,
    'nis_mean_correct_variance': float(np.mean(nis)),
    'nis_mean_overconfident_variance': float(np.mean(bad_nis)),
    'nis_95pct_coverage_correct_variance': nis_coverage,
}
checks = {
    'mean_nees_correct_close_to_dim_3': abs(correct_mean - 3.) < .20,
    'nees_95pct_coverage_near_95pct': .93 <= coverage <= .97,
    'overconfident_covariance_detected': wrong_mean > 12.,
    'mean_nis_correct_close_to_1': abs(float(np.mean(nis)) - 1.) < .05,
    'nis_95pct_coverage_near_95pct': .93 <= nis_coverage <= .97,
    'overconfident_innovation_variance_detected': float(np.mean(bad_nis)) > 7.,
}
result['checks'] = checks
result['pass'] = all(checks.values())
p = Path(__file__).resolve().with_name('benchmark_consistency_mc.json')
p.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['pass'] else 1)
