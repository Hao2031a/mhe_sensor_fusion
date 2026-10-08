# MHE Statistical Covariance Calibration — Ground-Truth/CI workflow

This upgrade **does not change the MHE optimizer or fast covariance worker**.
Published x/y/yaw covariance uses the prior behavior by default; the new
calibration option is **off**, with identity factors `[1.0,1.0,1.0]`.

## Why the previous 600s test failed

The original synthetic truth produced mean NEES = **0.932**, nominal 95%
coverage **100%**. That passed Strict timing but failed the coverage gate. The
statistics suggested conservative covariance *on that synthetic trajectory*;
they did not provide justification to scale the physical-robot covariance.

The original test assigned motion phase via `last_phase` (callback arrival
rather than the pose timestamp) and permitted nearest truth points without
two-sided interpolation. Such mismatches can invalidate per-phase consistency.
The updated stress probe **waits for bracketing reference timestamps**, rejects
extrapolation/large gaps, and associates phase/cycle with aligned truth.

## New measurements

`ci-results/long_stress.json` now includes:

- `covariance_consistency.nominal_state_nees` for x, y, yaw (1D marginal NEES)
- `nominal_state_95pct_coverage` for each axis (1D chi-square 95% reference)
- `phase_nees` / `phase_95pct_coverage` (nominal + fault phases separately)
- `alignment_bracket_ms` and `alignment_nearest_ms` diagnostics
- `independent_cycle_bootstrap` (30-second block resampling; correlation-aware)
- Bounded `aligned_pose_records` for independent holdout evaluation

NEES is formed from **published** pose P and the aligned estimate/reference.
Pose NEES degrees-of-freedom 3 has ideal Gaussian mean 3; the three marginal
NEES values have mean 1 if their uncertainties are calibrated. The 95% chi-
square cutoffs are 7.8147 (joint) and 3.8415 (per-axis). *NEES observations
remain temporally correlated.* The bootstrap resamples **trajectory cycles**,
not samples, to avoid a misleadingly narrow confidence interval.

The `/mhe/nis_raw` values remain approximate and must not be treated as exact
chi-square innovations without the proper residual covariance and source
correlation model.

## Docker: build + normal tests (25-second stress)

From inside `~/diffbot_ws/src/mhe_sensor_fusion`:

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

## Long duration: strict timing and consistency

```bash
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -e MHE_STRESS_SECONDS=600 \
  -e MHE_STRESS_STRICT=1 \
  -e MHE_STRESS_CONSISTENCY_GATE=1 \
  -v "$(pwd)/ci-results:/artifacts" \
  mhe-fusion-ci /bin/bash /ws/src/mhe_sensor_fusion/ci/run_long_stress.sh
```

**The consistency gate can FAIL**. This means the estimator's covariance still
requires calibration; it is not a Python/Docker fault. A short 25-second stress
run is insufficient for meaningful consistency gating (requires >=5 independent
30-second cycles).

## Multi-seed (train + independent holdout; usually >=30 min for three 10-min seeds)

```bash
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -e MHE_STRESS_SECONDS=600 \
  -e MHE_STRESS_SEEDS=20261008,20261009,20261010 \
  -v "$(pwd)/ci-results:/artifacts" \
  mhe-fusion-ci /bin/bash /ws/src/mhe_sensor_fusion/ci/run_multiseed_stress.sh
```

Artifacts: `ci-results/seed-*/long_stress.json`, `ci-results/seed-*/long_stress_gate.json`,
`ci-results/covariance_calibration_proposal.json`. The proposal fits standard-
deviation factors on the first seeds and checks the last seed **without
reusing its data during fitting**. It never changes `config/mhe.yaml`.
Use `MHE_STRESS_CONSISTENCY_GATE=1` for an additional fail-closed statistical
regression gate on every seed.

## Optional runtime calibration after independent validation

```yaml
covariance:
  calibration:
    enabled: false
    pose_std_scales: [1.0, 1.0, 1.0]
```

If calibrated factors are eventually validated on **independent physical-robot
ground truth**, they can be enabled manually. Each factor is a standard-
deviation multiplier and is limited to `[0.5, 2.0]`. The covariance mapping is
`P_calibrated = D * P_published * D`, with `D=diag(sx,sy,syaw)`; this preserves
cross-correlation and positive semidefiniteness. It does not change the
optimized state or ROS timestamp.

**Never enable calibration only because a synthetic NEES gate becomes green.**
Collected ground truth must have separately validated clock alignment,
measurement frame, and non-circular provenance (not derived from wheel odom).

## Validation status

Local offline C++ and Python unit/regression tests can validate the statistics,
calibration formula, strict truth alignment and gate fail behavior. **The Docker
Ceres/ROS integration and 10–30 minute stress results must be rerun** on the
user's ROS 2 Kilted machine after installing this updated package. Previous
successes do not automatically transfer to this revision.
