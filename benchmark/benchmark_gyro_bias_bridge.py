#!/usr/bin/env python3
"""Seeded Monte-Carlo test of the Brownian gyro-bias bridge, not ROS/Gazebo."""
import json
import os
from pathlib import Path
import numpy as np

def case(rng, dt, q, npaths=18000, steps=120):
    dw = rng.normal(0, np.sqrt(dt/steps), size=(npaths,steps))
    w = np.cumsum(dw, axis=1)
    tau = np.arange(1,steps+1,dtype=float)/steps
    bridge = w - w[:,-1,None]*tau[None,:]
    area = (dt/steps)* (bridge.sum(axis=1)-0.5*bridge[:,-1])*q
    actual = float(np.var(area,ddof=1))
    expected = q*q*dt**3/12
    rel = abs(actual-expected)/expected
    assert rel < .065,(dt,q,rel)
    return {'dt_sec':dt,'q_bias_rw':q,'empirical_variance_rad2':actual,
            'theoretical_variance_rad2':expected,'relative_error':rel}

rng=np.random.default_rng(20261008)
rows=[case(rng,dt,q) for dt in (.005,.02,.075) for q in (.02,.3)]
report={'type':'synthetic_Brownian_bridge_variance_not_ros',
        'seed':20261008,'pass':True,'cases':len(rows),
        'max_relative_variance_error':max(r['relative_error'] for r in rows),
        'scenarios':rows}
out=Path(os.environ.get('CI_ARTIFACT_DIR',str(Path(__file__).parent)))
out.mkdir(parents=True,exist_ok=True)
(out/'benchmark_gyro_bias_bridge.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASS gyro bias Brownian bridge Monte Carlo',len(rows),'scenarios')
