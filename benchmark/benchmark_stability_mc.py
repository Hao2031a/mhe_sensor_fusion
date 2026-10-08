#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np

DT = 0.01
TRIALS = 200


def shape(targets, *, fixed_tau, max_accel, max_decel, max_jerk,
          median3, adaptive_tau, quiet_tau=0.0, fast_tau=0.0, transition=1.0):
    value = float(targets[0])
    accel = 0.0
    hist = []
    out = np.zeros_like(targets, dtype=float)
    for k, raw in enumerate(targets):
        hist.append(float(raw))
        if len(hist) > 3:
            hist.pop(0)
        target = float(np.median(hist)) if median3 and len(hist) == 3 else float(raw)
        tau = fixed_tau
        if adaptive_tau:
            r = np.clip(abs(target - value) / max(abs(transition), 1e-6), 0.0, 1.0)
            s = r*r*(3.0 - 2.0*r)
            tau = quiet_tau + (fast_tau - quiet_tau)*s
        alpha = 1.0 if tau <= 1e-6 else DT/(tau + DT)
        ft = value + alpha*(target-value)
        desired = (ft-value)/DT
        slowing = (value*target < 0.0) or (abs(target) < abs(value))
        lim = max_decel if slowing else max_accel
        desired = float(np.clip(desired, -lim, lim))
        mda = max_jerk*DT
        accel += float(np.clip(desired-accel, -mda, mda))
        prev = value
        value += accel*DT
        if (target-prev)*(target-value) <= 0.0:
            value = target
            accel = 0.0
        out[k] = value
    return out


def t90(t, y, start, initial, target):
    idx0 = int(round(start/DT))
    level = initial + 0.9*(target-initial)
    hits = np.where(y[idx0:] >= level)[0] if target >= initial else np.where(y[idx0:] <= level)[0]
    return np.nan if hits.size == 0 else float(t[idx0+hits[0]]-start)


def run_trial(kind, seed):
    rng = np.random.default_rng(seed)
    if kind == 'linear':
        t = np.arange(0.0, 12.0, DT)
        truth = np.zeros_like(t)
        truth[(t>=1)&(t<5)] = 0.30
        truth[(t>=5)&(t<8)] = 0.15
        truth[t>=8] = 0.35
        raw = truth + rng.normal(0, 0.012, t.size)
        amp = 0.08
        windows = [(2,4.5),(5.8,7.5),(8.8,11.5)]
        base = dict(fixed_tau=.030,max_accel=3,max_decel=4,max_jerk=60,median3=False,adaptive_tau=False)
        new = dict(fixed_tau=.030,max_accel=3,max_decel=4,max_jerk=60,median3=True,adaptive_tau=True,
                   quiet_tau=.060,fast_tau=.020,transition=.060)
        step=(1.0,0.0,0.30)
    else:
        t = np.arange(0.0, 10.0, DT)
        truth = np.zeros_like(t)
        truth[(t>=1)&(t<4)] = 0.80
        truth[(t>=4)&(t<7)] = -0.40
        truth[t>=7] = 1.00
        raw = truth + rng.normal(0, 0.025, t.size)
        amp = 0.18
        windows=[(2,3.5),(5,6.5),(8,9.5)]
        base=dict(fixed_tau=.018,max_accel=12,max_decel=16,max_jerk=240,median3=False,adaptive_tau=False)
        new=dict(fixed_tau=.018,max_accel=12,max_decel=16,max_jerk=240,median3=True,adaptive_tau=True,
                 quiet_tau=.040,fast_tau=.008,transition=.150)
        step=(1.0,0.0,0.80)
    pool=np.arange(int(1.5/DT), t.size-int(.3/DT))
    for idx in rng.choice(pool,size=20,replace=False):
        raw[idx]+=rng.choice([-1,1])*amp
    yb=shape(raw,**base); yn=shape(raw,**new)
    mask=np.zeros_like(t,dtype=bool)
    for a,b in windows: mask|=(t>=a)&(t<b)
    sb=float(np.std(yb[mask]-truth[mask])); sn=float(np.std(yn[mask]-truth[mask]))
    tb=t90(t,yb,*step); tn=t90(t,yn,*step)
    return {
        'jitter_improvement_pct': (1-sn/max(sb,1e-12))*100,
        't90_penalty_s': tn-tb,
        'max_step_ratio': float(np.max(np.abs(np.diff(yn))) / max(np.max(np.abs(np.diff(yb))),1e-12)),
    }


def summarize(kind, seed0):
    rows=[run_trial(kind,seed0+i) for i in range(TRIALS)]
    ji=np.array([r['jitter_improvement_pct'] for r in rows])
    tp=np.array([r['t90_penalty_s'] for r in rows])
    mr=np.array([r['max_step_ratio'] for r in rows])
    out={
        'trials': TRIALS,
        'jitter_improvement_pct_median': float(np.median(ji)),
        'jitter_improvement_pct_p05': float(np.percentile(ji,5)),
        'jitter_improvement_pct_p95': float(np.percentile(ji,95)),
        't90_penalty_s_median': float(np.nanmedian(tp)),
        't90_penalty_s_p95': float(np.nanpercentile(tp,95)),
        'max_step_ratio_median': float(np.median(mr)),
        'max_step_ratio_p95': float(np.percentile(mr,95)),
    }
    out['pass']=bool(
        out['jitter_improvement_pct_p05'] >= 15.0 and
        out['t90_penalty_s_p95'] <= 0.0200001 and
        out['max_step_ratio_p95'] <= 1.05
    )
    return out

report={
    'linear': summarize('linear',1000),
    'angular': summarize('angular',5000),
}
report['overall_pass']=bool(report['linear']['pass'] and report['angular']['pass'])
print(json.dumps(report,indent=2))
out=Path(__file__).resolve().parent/'benchmark_stability_mc.json'
out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(f'Saved benchmark results to: {out}')
if not report['overall_pass']:
    raise SystemExit(2)
