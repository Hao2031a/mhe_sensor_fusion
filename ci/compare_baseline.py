#!/usr/bin/env python3
"""Compare observed full-solver wall timing against a user-provided pre-change run.
Not a controlled A/B test unless both runs use identical hardware and inputs.
"""
import argparse
import json
from pathlib import Path


def compare(baseline, current):
    before = baseline['solver_time_ms']
    after = current['solver_time_ms']
    changes = {}
    for name in ('p50', 'p95', 'p99', 'max'):
        old = before.get(name)
        new = after.get(name)
        changes[name] = {'before_ms': old, 'after_ms': new,
                         'change_percent': 100 * (new / old - 1) if old and new is not None else None}
    return {'comparison_type': 'observational',
            'note': 'Negative change_percent means improved speed. Confirm same CPU, load, ROS image and test conditions before causal claims.',
            'solver_latency': changes}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--current', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    data = compare(json.loads(args.baseline.read_text()),
                   json.loads(args.current.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps(data, indent=2))


if __name__ == '__main__':
    main()
