# Stability Optimization Benchmark

This report is generated from the current package configuration after adding the
median-of-3 + motion-adaptive output target guard.

## Core estimator benchmark

- Common slip truth: 0.20
- Peak estimate: 0.2084
- End-of-excitation estimate: 0.1643
- Mean hold 4-6 s: 0.1484
- Mean hold 6-8 s: 0.1215
- Accelerometer robustness improvement: 93.84x RMSE reduction in the synthetic offset/spike stress case
- Hessian covariance relative error: 1.12% for N=5 and 0.64% for N=20 in the scalar Gaussian consistency test
- Batched solve-count reduction: 33.3% nominal, 75% for an observed four-event tick

## Deterministic output benchmark

Linear channel:
- fixed-tau baseline steady error std: 0.0057203 m/s
- candidate steady error std: 0.0038474 m/s
- jitter reduction: 32.74%
- t90: unchanged at 0.12 s
- maximum one-cycle step: unchanged at 0.03 m/s

Angular channel:
- fixed-tau baseline steady error std: 0.015096 rad/s
- candidate steady error std: 0.0096695 rad/s
- jitter reduction: 35.95%
- t90: unchanged at 0.08 s
- maximum one-cycle step: unchanged at 0.16 rad/s

## Monte-Carlo stability benchmark (200 trials/channel)

Linear:
- median jitter improvement: 28.42%
- 5th percentile jitter improvement: 19.41%
- 95th percentile step-response penalty: 10 ms
- 95th percentile max-step ratio vs baseline: 1.024
- PASS

Angular:
- median jitter improvement: 36.91%
- 5th percentile jitter improvement: 16.22%
- 95th percentile step-response penalty: 10 ms
- 95th percentile max-step ratio vs baseline: 1.000
- PASS

Overall: **PASS**

These are offline synthetic/regression benchmarks of the implemented equations and output
shaper. They do not replace on-robot ROS/Gazebo timing and CPU measurements.
