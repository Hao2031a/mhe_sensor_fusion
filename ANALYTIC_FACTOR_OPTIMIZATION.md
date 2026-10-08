# MHE analytic factor optimization — Gazebo/ROS 2 Kilted

## Scope

This is a **numerical computation optimization**, not a new physical sensor model.
The 9-state MHE, sensor timestamps, horizon, Ceres residual values, robust Huber
losses, adaptive R/Q, slip observability, yaw low-latency bypass, timestamp-aligned
output, covariance worker, QR fallback and TF frames are unchanged.

Only two hot factors can now use analytic rather than Ceres Jet AutoDiff Jacobians:

- `ProcessCost`: 9 residuals, two 9-state blocks (18 independent variables).
- `WheelPairCost`: 2 residuals, one 9-state block.

The exact reference AutoDiff factors remain in source and are selected by
`solver.analytic_factors_enabled: false`. The fast version is enabled by default.
Both branches have the same Ceres cost function values and the same loss functions.

## Mathematics

Let `d=dt`, `v_m=(v0+v1)/2`, `w_m=(w0+w1)/2`,
`theta_m=theta0+d*w_m/2`.

```
rx = (x1-x0-d*v_m*cos(theta_m))/sigma_x
ry = (y1-y0-d*v_m*sin(theta_m))/sigma_y
rtheta = (theta1-theta0-d*w_m)/sigma_theta
r[i] = (state1[i]-state0[i])/sigma[i] for i=v,w,bg,ba,sl,sr
```

Examples of exact Jacobian entries:

```
drx/dtheta0 = (d*v_m*sin(theta_m))/sigma_x
drx/dw0 = drx/dw1 = (d*d*v_m*sin(theta_m))/(4*sigma_x)
dry/dtheta0 = -(d*v_m*cos(theta_m))/sigma_y
dry/dw0 = dry/dw1 = -(d*d*v_m*cos(theta_m))/(4*sigma_y)
drtheta/dw0 = drtheta/dw1 = -d/(2*sigma_theta)
```

Wheel measurements:

```
l = v-(B/2)*w, r=v+(B/2)*w
r_left = ((1+sl)*l-wheel_left)/sigma_left
r_right = ((1+sr)*r-wheel_right)/sigma_right
```

Analytic process Jacobians have 30 nonzero entries across two 9x9 blocks;
wheel Jacobian has six nonzero entries across a 2x9 block. Ceres still receives
full row-major blocks (zero-filled for unused derivatives).

## Tests and reproducibility

Offline no-ROS tests executed in the authoring environment:

- 1,200 randomized test horizons, 426,000 reference-residual, finite-difference
  Jacobian, partial-Jacobian and no-Jacobian assertions PASS.
- 34 Python regression/unit tests PASS.
- `benchmark/run_benchmarks.sh` PASS.
- `ci/check_regressions.py` PASS.
- All 3 parameter YAMLs parsed and `solver.analytic_factors_enabled` nested correctly.

**Not executed here:** colcon build, actual Ceres node, ROS DDS smoke, Docker,
Gazebo ground truth, Nav2-load RT benchmarks. Container lacks ROS/Ceres/Eigen.

Docker CI adds `test_ceres_analytic`: 24 identical 13-state horizons run through
Ceres with analytic and AutoDiff versions. Acceptance requires
max absolute final-state difference < 1e-5 and max final-cost difference < 1e-7.
It also records median Ceres solver time for each variant to
`ci-results/ceres_analytic_ab.json`. Timing is **advisory** on shared runners;
there is no unverified claim of performance speedup.

Run in ROS package root:

```
colcon build --packages-select mhe_sensor_fusion
```

Docker CI:

```
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Compare Gazebo A/B with exactly the same recording/sensor stimulation and
`solver.analytic_factors_enabled: false` vs `true`. One MHE TF publisher at a time.
Inspect `/mhe/rt_profile` indices: build=0, Ceres=1, covariance=2,
marginalization=3, solve_total=4, callback=5; correlate with yaw RMSE from
independent Gazebo model-pose ground truth. Keep angular smoothing bypass on.
Do not interpret `ros2 topic hz` using wall time as sim-time frequency when
Gazebo real-time factor differs from 1.

## Safety/rollout

- Roll back immediately by setting `solver.analytic_factors_enabled: false`.
- Do not tune sigma/endpoint guards and the numerical backend in the same A/B.
- No solve accuracy improvement is claimed: the optimization preserves the
  residual model. Its intended gain is lower derivative computation cost.
- Full ROS/Ceres and Gazebo tests are required before operational deployment.
