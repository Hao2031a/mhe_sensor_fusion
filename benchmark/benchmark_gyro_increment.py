#!/usr/bin/env python3
"""Standalone synthetic mathematical QA: gyro increment closure and conservative noise.
This is NOT a Gazebo yaw-RMSE comparison and has no Ceres/ROS runtime claims.
"""
import json
import math
from pathlib import Path
import numpy as np
rng=np.random.default_rng(20261008)
results=[]
for hz in (40,55,100):
 for angular_accel in (0,0.5,1.5,3.):
  n=25000
  dt=np.full(n,1/hz)
  w0=rng.uniform(-2,2,n)
  bias0=rng.uniform(-.07,.07,n)
  bg1=bias0+rng.normal(0,.0001,n)
  # The gyro sample is taken at interval END (backward integration rule).
  w_end=w0+angular_accel*dt
  measured_gyro=w_end+.5*(bias0+bg1)+rng.normal(0,.01,n)
  actual_delta=(w0+.5*angular_accel*dt)*dt
  corrected_delta=(measured_gyro-.5*(bias0+bg1))*dt
  err=corrected_delta-actual_delta
  sigma=dt*np.hypot(.01,.12)
  coverage=float(np.mean(np.abs(err)<=1.96*sigma))
  rms=float(np.sqrt(np.mean(err**2)))
  predicted_sigma=float(np.mean(sigma))
  assert coverage>=.94, (hz,angular_accel,coverage)
  results.append(dict(sensor_hz=hz,angular_acceleration_rad_s2=angular_accel,
    rmse_rad=rms,mean_predicted_sigma_rad=predicted_sigma,
    within_95pct_noise_envelope=coverage))
report={'description':'Synthetic end-sample gyro quadrature at constant angular acceleration',
 'pass':True,'trials_per_scenario':25000,'results':results,
 'warning':'Not a Gazebo ground-truth or ROS/Ceres runtime benchmark; 0.12 rad/s model rate sigma is a tunable conservative envelope.'}
path=Path(__file__).with_suffix('.json');path.write_text(json.dumps(report,indent=2)+'\n')
print('PASS gyro increment synthetic noise envelope:',len(results),'scenarios; min coverage:',min(x['within_95pct_noise_envelope'] for x in results))
