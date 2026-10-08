#!/usr/bin/env python3
"""Timestamp-aware pose-covariance consistency diagnostics.

These helpers work with both synthetic truth and independently recorded truth.
Statistical tests assume an *appropriately* calibrated innovation/covariance model;
synthetic trajectories alone cannot certify physical-robot calibration.
"""
import math
import random
from collections import deque
import numpy as np

CHI2_3_95 = 7.814727903
CHI2_1_95 = 3.841458821


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_covariance_3x3(cov):
    a = np.asarray(cov, dtype=float).reshape((6, 6))
    return a[np.ix_([0, 1, 5], [0, 1, 5])]


def pose_error(estimate, truth):
    return np.array([estimate[0] - truth[0],
                     estimate[1] - truth[1],
                     wrap(estimate[2] - truth[2])], dtype=float)


def nees_pose(estimate, truth, covariance):
    """Return (3D NEES, smallest eigenvalue). Reject non-PD or invalid P."""
    mat = pose_covariance_3x3(covariance)
    if not np.all(np.isfinite(mat)) or not np.allclose(mat, mat.T, atol=1.e-7):
        return None, float('nan')
    eig = np.linalg.eigvalsh(mat)
    min_eigenvalue = float(eig[0])
    if min_eigenvalue <= 1.e-12:
        return None, min_eigenvalue
    err = pose_error(estimate, truth)
    return float(err @ np.linalg.solve(mat, err)), min_eigenvalue


def nees_statewise(estimate, truth, covariance):
    """Marginal one-DOF NEES for x, y and yaw using the *published* P."""
    p = pose_covariance_3x3(covariance)
    if not np.all(np.isfinite(p)) or min(np.diag(p)) <= 1.e-12:
        return None
    e = pose_error(estimate, truth)
    return tuple(float((e[i] ** 2) / p[i, i]) for i in range(3))


class BoundedSamples:
    """Deterministically downsample to keep 30-minute stress tests bounded."""
    def __init__(self, stride=1, max_samples=25000):
        self.stride = max(1, int(stride))
        self.max_samples = max_samples
        self.count = 0
        self.samples = []

    def add(self, value):
        if not math.isfinite(value):
            return
        self.count += 1
        if self.count % self.stride == 0:
            if len(self.samples) >= self.max_samples:
                self.samples = self.samples[::2]
                self.stride *= 2
            self.samples.append(float(value))

    def stats(self):
        v = sorted(self.samples)
        if not v:
            return {'count': self.count, 'sampled': 0, 'p50': None, 'p95': None,
                    'p99': None, 'max': None, 'mean': None}
        at = lambda f: v[min(len(v) - 1, round((len(v) - 1) * f))]
        return {'count': self.count, 'sampled': len(v), 'p50': at(.5),
                'p95': at(.95), 'p99': at(.99), 'max': v[-1],
                'mean': sum(v) / len(v)}


class PoseTruthBuffer:
    """Timestamped reference interpolator; new API requires two-sided support."""
    def __init__(self, size=1000):
        self.samples = deque(maxlen=size)

    def add(self, stamp_ns, x, y, yaw, valid=True, phase='unknown', cycle=-1):
        self.samples.append((stamp_ns, x, y, yaw, valid, phase, cycle))

    def lookup_aligned(self, stamp_ns, max_bracket_ms=30.0):
        """Reject extrapolation, wide brackets and ambiguous phase transitions.

        Return a dict with precisely interpolated pose, aligned phase and
        matching metadata. None means that reference is not yet bracketed.
        """
        if not self.samples or stamp_ns < self.samples[0][0] or stamp_ns > self.samples[-1][0]:
            return None
        a = list(self.samples)
        lo, hi = 0, len(a) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if a[mid][0] < stamp_ns:
                lo = mid + 1
            else:
                hi = mid
        upper = a[lo]
        if upper[0] == stamp_ns:
            return {'pose': tuple(upper[1:4]), 'valid': bool(upper[4]),
                    'phase': upper[5], 'cycle': upper[6],
                    'bracket_ms': 0., 'nearest_ms': 0.}
        if lo == 0:
            return None
        lower = a[lo - 1]
        span_ns = upper[0] - lower[0]
        if span_ns <= 0 or span_ns > max_bracket_ms * 1e6:
            return None
        alpha = (stamp_ns - lower[0]) / span_ns
        phase_agrees = lower[5] == upper[5] and lower[6] == upper[6]
        return {
            'pose': (lower[1] + (upper[1] - lower[1]) * alpha,
                     lower[2] + (upper[2] - lower[2]) * alpha,
                     wrap(lower[3] + wrap(upper[3] - lower[3]) * alpha)),
            'valid': bool(lower[4] and upper[4] and phase_agrees),
            'phase': lower[5] if phase_agrees else 'transition',
            'cycle': lower[6] if phase_agrees else -1,
            'bracket_ms': span_ns * 1e-6,
            'nearest_ms': min(stamp_ns - lower[0], upper[0] - stamp_ns) * 1e-6,
        }

    def lookup(self, stamp_ns, tolerance_ns=40_000_000):
        """Legacy four-tuple nearest/interpolated lookup for existing callers."""
        sample = self.lookup_aligned(stamp_ns, max_bracket_ms=2*tolerance_ns*1e-6)
        if sample is not None:
            return (*sample['pose'], sample['valid'])
        if not self.samples:
            return None
        # Keep preexisting closest-point semantics for external callers only.
        a = min(self.samples, key=lambda v: abs(v[0] - stamp_ns))
        if abs(a[0] - stamp_ns) <= tolerance_ns:
            return (*a[1:4], a[4])
        return None


def cycle_block_bootstrap(cycles, replicates=600, seed=108):
    """Bootstrap by 30s trajectory cycle, NOT by serially correlated samples.

    ``cycles``: mapping cycle_id -> {'n': int, 'sum_nees': float, 'covered': int}.
    At least five complete-ish blocks are needed for an informative interval.
    """
    blocks = [v for v in cycles.values() if v['n'] >= 10]
    if len(blocks) < 5:
        return {'independent_blocks': len(blocks), 'valid': False,
                'mean_nees_ci95': None, 'coverage_ci95': None}
    rng = random.Random(seed)
    means, coverages = [], []
    for _ in range(replicates):
        sample = [blocks[rng.randrange(len(blocks))] for _ in blocks]
        n = sum(s['n'] for s in sample)
        means.append(sum(s['sum_nees'] for s in sample) / n)
        coverages.append(sum(s['covered'] for s in sample) / n)
    def ci(a):
        a.sort()
        return [a[int(.025 * (len(a)-1))], a[int(.975 * (len(a)-1))]]
    return {'independent_blocks': len(blocks), 'valid': True,
            'mean_nees_ci95': ci(means), 'coverage_ci95': ci(coverages)}
