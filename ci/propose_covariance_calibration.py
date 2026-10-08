#!/usr/bin/env python3
"""Offline, holdout-only pose covariance calibration proposal.

Train on at least 2 independent-seed ROS stress runs; validate on an
independent seed. Never modifies mhe.yaml, and never declares physical-robot
covariance calibrated from synthetic ground truth.

Usage:
  python3 ci/propose_covariance_calibration.py --train seed1.json seed2.json \
     --holdout seed3.json --output ci-results/calibration_proposal.json
"""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from consistency_metrics import CHI2_1_95, CHI2_3_95


def extract_records(path):
    r = json.loads(Path(path).read_text())
    if r.get('synthetic_ground_truth_only') is not True:
        raise ValueError(f'{path}: this tool currently supports synthetic-truth stress report only')
    records = r.get('covariance_consistency', {}).get('aligned_pose_records', [])
    filtered = []
    for rec in records:
        if not rec.get('nominal'):
            continue
        e, p = np.asarray(rec.get('error'), float), np.asarray(rec.get('P'), float)
        if e.shape != (3,) or p.shape != (3,3) or not np.isfinite(e).all() or not np.isfinite(p).all():
            continue
        if np.linalg.eigvalsh(p).min() <= 1.e-12:
            continue
        filtered.append((e,p,rec['cycle']))
    if len(filtered) < 100:
        raise ValueError(f'{path}: not enough independent nominal aligned pose samples')
    return int(r['seed']), filtered


def evaluate(records, scales):
    d = np.diag(scales)
    nees, per_state = [], [[], [], []]
    for e,p,_ in records:
        q = d @ p @ d
        nees.append(float(e @ np.linalg.solve(q,e)))
        for i in range(3):
            per_state[i].append(float(e[i]**2/q[i,i]))
    return {'count':len(nees),'nees_mean':float(np.mean(nees)),
            'nees_95pct_coverage':float(np.mean(np.asarray(nees) <= CHI2_3_95)),
            'state_nees_mean':[float(np.mean(a)) for a in per_state],
            'state_95pct_coverage':[float(np.mean(np.asarray(a) <= CHI2_1_95)) for a in per_state]}


def propose(train, holdout):
    train_sets = [extract_records(p) for p in train]
    holdout_seed, validation = extract_records(holdout)
    train_seeds = [sid for sid, _ in train_sets]
    if len(train_seeds) < 2 or len(set(train_seeds)) != len(train_seeds) or holdout_seed in train_seeds:
        raise ValueError('Need >=2 distinct training seeds and an independent holdout seed')
    flat = [rec for _, records in train_sets for rec in records]
    nees_means = [float(np.mean([e[i]**2 / p[i,i] for e,p,_ in flat])) for i in range(3)]
    # Regularized toward identity, not fitted on validation data. Bound std
    # changes to a factor of two; an observed discrepancy is not proof.
    raw_scales = [math.sqrt(max(.0, v)) for v in nees_means]
    scales = [round(min(2.0, max(0.5, 1 + 0.50*(v-1))), 6) for v in raw_scales]
    base = evaluate(validation, [1.,1.,1.])
    after = evaluate(validation, scales)
    def distance(m):
        # Diagnostic mismatch to reference only, not an optimality guarantee.
        return abs(m['nees_mean']-3.) + 6.0 * abs(m['nees_95pct_coverage']-.95)
    better = distance(after) < distance(base)
    # Coarse held-out safety envelope; do not equate it to real calibration.
    in_envelope = (0.5 <= after['nees_mean'] <= 6.0 and
                   0.80 <= after['nees_95pct_coverage'] <= 0.995 and
                   all(0.20 <= v <= 5.0 for v in after['state_nees_mean']) and
                   all(0.75 <= v <= 0.995 for v in after['state_95pct_coverage']))
    return {
        'candidate_only':True, 'automatically_applied_to_mhe':False,
        'training_seeds':train_seeds, 'holdout_seed':holdout_seed,
        'training_axis_nees_mean':nees_means, 'candidate_pose_std_scales':scales,
        'holdout_baseline':base, 'holdout_candidate':after,
        'holdout_improves_consistency_distance':better,
        'holdout_passes_diagnostic_envelope':in_envelope,
        'eligible_for_review':better and in_envelope,
        'warning': 'Synthetic holdout results do NOT authorize enabling pose covariance calibration on a real robot; use independent physical ground truth.'
    }


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--train',type=Path,nargs='+',required=True)
    ap.add_argument('--holdout',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    r=propose(args.train,args.holdout)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps(r,indent=2))
    # Do not fail Docker's normal runtime regression gate because a
    # diagnostic calibration proposal is not eligible for deployment.

if __name__=='__main__':
    main()
