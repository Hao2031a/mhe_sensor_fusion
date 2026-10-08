#!/usr/bin/env python3
"""Deterministic mathematical yaw/SE2 regression, not a ROS/Gazebo test."""
import json
import math
from pathlib import Path

PI=math.pi

def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))

def exact_arc(yaw,v,w,dt):
    u=0.5*w*dt
    sinc=1-u*u/6+u**4/120-u**6/5040 if abs(u)<1e-3 else math.sin(u)/u
    m=v*dt*sinc
    return m*math.cos(yaw+u),m*math.sin(yaw+u)

def midpoint(yaw,v,w,dt):
    mid=yaw+0.5*w*dt
    return v*dt*math.cos(mid),v*dt*math.sin(mid)

def run():
    scenarios=[]
    for hz in (10,20,55,100):
        dt=1/hz
        for w in (0.0,0.5,2.0,4.0):
            errors=[]
            exact_errors=[]
            yaw=0.0
            for k in range(300):
                v=0.2+0.1*math.sin(0.04*k)
                true=exact_arc(yaw,v,w,dt)
                old=midpoint(yaw,v,w,dt)
                # Independent numerical quadrature, with midpoint of 256 substeps.
                refx=refy=0.0
                for j in range(256):
                    th=yaw+w*dt*(j+0.5)/256
                    refx+=v*dt/256*math.cos(th)
                    refy+=v*dt/256*math.sin(th)
                errors.append(math.hypot(old[0]-refx,old[1]-refy))
                exact_errors.append(math.hypot(true[0]-refx,true[1]-refy))
                yaw=wrap(yaw+w*dt)
            scenarios.append({'sensor_hz':hz,'yaw_rate_rad_s':w,
               'midpoint_step_rmse_m':math.sqrt(sum(e*e for e in errors)/len(errors)),
               'exact_arc_step_rmse_m':math.sqrt(sum(e*e for e in exact_errors)/len(exact_errors))})
    residual_cases=[]
    for initial,final in [(PI-0.002,-PI+0.003),(-PI+0.002,PI-0.003),
                          (0.3,0.3+2*PI),(0.3,0.3-2*PI)]:
        residual_cases.append({'legacy_abs_rad':abs(final-initial),
                               'so2_abs_rad':abs(wrap(final-initial))})
    max_arc=max(s['exact_arc_step_rmse_m'] for s in scenarios)
    high=[s for s in scenarios if s['yaw_rate_rad_s']>=2.0]
    max_ratio=max(s['exact_arc_step_rmse_m']/(s['midpoint_step_rmse_m']+1e-30) for s in high)
    passed=max_arc<2e-8 and max_ratio<0.005 and all(r['so2_abs_rad']<0.006 for r in residual_cases)
    report={'pass':passed,'type':'independent_numerical_reference_synthetic',
            'mathematical_not_yaw_rmse_claim':True,
            'max_exact_arc_rmse_m':max_arc,
            'max_exact_to_midpoint_error_ratio_high_turn':max_ratio,
            'yaw_wrap_cases':residual_cases,'scenarios':scenarios}
    p=Path(__file__).with_suffix('.json');p.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'pass':passed,'max_exact_arc_rmse_m':max_arc,
          'max_error_ratio_high_turn':max_ratio,'scenarios':len(scenarios)},indent=2))
    if not passed: raise SystemExit(1)
if __name__=='__main__':run()
