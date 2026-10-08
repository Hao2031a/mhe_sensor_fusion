#!/usr/bin/env python3
"""Offline numerical test of the 9x9 chain-Schur algorithm (no ROS/Ceres).

Only arithmetic performance of a NumPy reference implementation is measured;
this is NOT a wall-time benchmark of the C++ ROS node.
"""
import json
import os
import time
from pathlib import Path
import numpy as np

rng = np.random.default_rng(20261008)
trials = 0
max_h_error = 0.
max_b_error = 0.
records = []
for n in (2, 3, 4, 5, 7, 9):
    for trial in range(6):
        D, E, b = [], [], []
        H = np.zeros((n*9,n*9)); rhs = np.zeros(n*9)
        for i in range(n):
            A = rng.normal(size=(9,9))
            Di = A.T@A + np.eye(9)*30
            bi = rng.normal(size=9)
            D.append(Di); b.append(bi)
            H[9*i:9*(i+1), 9*i:9*(i+1)]=Di
            rhs[9*i:9*(i+1)]=bi
            if i > 0:
                Ei=rng.normal(size=(9,9))*0.04
                E.append(Ei)
                H[9*(i-1):9*i, 9*i:9*(i+1)]=Ei
                H[9*i:9*(i+1),9*(i-1):9*i]=Ei.T
        for remove in range(1,n):
            t0=time.perf_counter()
            h, g = D[0].copy(), b[0].copy()
            for i in range(remove):
                inv_edge=np.linalg.solve(h+np.eye(9)*1e-8, E[i])
                inv_rhs=np.linalg.solve(h+np.eye(9)*1e-8, g)
                h=D[i+1]-E[i].T@inv_edge
                g=b[i+1]-E[i].T@inv_rhs
            tb=time.perf_counter()-t0
            ne=remove*9
            t0=time.perf_counter()
            haa=H[:ne,:ne]+np.eye(ne)*1e-8
            hab=H[:ne,ne:ne+9]
            ref_h=H[ne:ne+9,ne:ne+9]-hab.T@np.linalg.solve(haa,hab)
            ref_b=rhs[ne:ne+9]-hab.T@np.linalg.solve(haa,rhs[:ne])
            td=time.perf_counter()-t0
            herr=float(np.max(np.abs(h-ref_h)))
            berr=float(np.max(np.abs(g-ref_b)))
            max_h_error=max(max_h_error,herr)
            max_b_error=max(max_b_error,berr)
            assert herr<1e-8 and berr<1e-8,(n,remove,herr,berr)
            records.append((tb,td))
            trials+=1
summary={'type':'synthetic_numeric_reference_not_cpp_ros', 'seed':20261008,
         'cases':trials,'max_abs_h_error':max_h_error,
         'max_abs_rhs_error':max_b_error,
         'block_median_ms_numpy':float(np.median([q[0] for q in records])*1000),
         'dense_median_ms_numpy':float(np.median([q[1] for q in records])*1000),
         'pass':True}
out=Path(os.environ.get('CI_ARTIFACT_DIR',str(Path(__file__).parent)))
out.mkdir(parents=True,exist_ok=True)
(out/'benchmark_block_schur.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
