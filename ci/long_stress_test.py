#!/usr/bin/env python3
"""Bounded-memory long-run test of the actual ROS 2 MHE node.

This runner publishes deterministic, timestamped SYNTHETIC sensor messages with
independently integrated synthetic pose truth. The resulting NEES/approximate
NIS diagnostics are NOT evidence of physical-robot calibration.

Usage:
 python3 ci/long_stress_test.py --seconds 600 --output /artifacts/long_stress.json
"""
import argparse
import json
import math
import os
import random
import statistics
import time
from collections import Counter, deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

from consistency_metrics import (BoundedSamples, PoseTruthBuffer, nees_pose, nees_statewise,
                                 cycle_block_bootstrap, CHI2_3_95, CHI2_1_95, wrap)


PERIOD = 30.0


def motion(t):
    """True robot velocity, yaw rate and scenario name in a repeating 30s loop."""
    p = t % PERIOD
    if p < 2:
        return 0., 0., 'idle'
    if p < 4:
        return 0.15 * (p - 2) / 2, 0., 'acceleration'
    if p < 9:
        return 0.15, 0., 'straight'
    if p < 14:
        return 0.13, 0.25 * math.sin((p - 9) * math.pi / 5), 'turn'
    if p < 18:
        return 0.16, 0., 'slip'
    if p < 20:
        return 0.10, 0., 'imu_dropout'
    if p < 22:
        return 0.10, 0., 'wheel_dropout'
    if p < 26:
        return 0.10, 0., 'recovery'
    return 0., 0., 'stop'


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class StressProbe(Node):
    def __init__(self, seed=20261008):
        super().__init__('mhe_long_stress_probe')
        self.rng = random.Random(seed)
        self.pub_wheel = self.create_publisher(Odometry, '/odom/unfiltered', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/mhe_ci/odom', self.on_odom, 100)
        self.create_subscription(Float64MultiArray, '/mhe/rt_profile', self.on_rt, 100)
        self.create_subscription(Float64MultiArray, '/mhe/cov_worker_status', self.on_worker, 100)
        self.create_subscription(Float64MultiArray, '/mhe/gating_status', self.on_gating, 100)
        self.create_subscription(Float64MultiArray, '/mhe/nis_raw', self.on_nis, 100)
        self.create_subscription(Float64MultiArray, '/mhe/timing', self.on_timing, 100)
        self.truth = PoseTruthBuffer(size=1000)
        self.true_pose = [0., 0., 0.]
        self.previous_elapsed = None
        self.last_true_v = 0.
        self.last_stamp = None
        self.odom_count = 0
        self.odom_nonfinite = 0
        self.odom_nonmonotonic = 0
        self.max_odom_gap_ms = 0.
        self.cov_not_psd = 0
        self.nees = BoundedSamples(stride=1)
        self.nees_normal = BoundedSamples(stride=1)
        self.abs_xy_error = BoundedSamples(stride=1)
        self.abs_yaw_error = BoundedSamples(stride=1)
        self.covered_nees = 0
        self.per_phase = Counter()
        self.unmatched_truth = 0
        self.last_worker = None
        self.worker_seen = 0
        self.cov_worker_age = BoundedSamples(stride=10)
        self.last_profile = None
        self.first_profile = None
        self.solver_sample = BoundedSamples(stride=1)
        self.callback_sample = BoundedSamples(stride=1)
        self.period_sample = BoundedSamples(stride=1)
        self.last_solve_seq = -1
        self.callback_profiles = 0
        self.nis = [BoundedSamples(stride=20) for _ in range(4)]
        self.nis_greater_95 = [0] * 4
        self.nis_count = [0] * 4
        self.gating = None
        self.timing = None
        self.imu_published = 0
        self.wheel_published = 0
        self.injected = Counter()
        self.last_phase = 'idle'
        self.last_odom_yaw = 0.
        self.last_cycle_spike = -1
        self.last_cycle_out_of_order = -1
        self.truth_pending = deque(maxlen=64)
        self.state_nees_nominal = {name: BoundedSamples() for name in ('x', 'y', 'yaw')}
        self.state_coverage_nominal = Counter()
        self.phase_nees = {}
        self.phase_coverage = Counter()
        self.cycle_nees = {}
        self.truth_bracket_ms = BoundedSamples()
        self.truth_nearest_ms = BoundedSamples()
        self.seed = seed
        # Bounded 3.3Hz timestamp-aligned pose-error/P samples permit held-out
        # evaluation of D * P * D without modifying the live estimator.
        self.calibration_records = []

    def on_odom(self, msg):
        self.odom_count += 1
        s = stamp_ns(msg)
        if self.last_stamp is not None:
            if s <= self.last_stamp:
                self.odom_nonmonotonic += 1
            else:
                self.max_odom_gap_ms = max(self.max_odom_gap_ms,
                                           (s - self.last_stamp) * 1e-6)
        self.last_stamp = s
        quat = msg.pose.pose.orientation
        yaw = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y),
                         1 - 2 * (quat.y * quat.y + quat.z * quat.z))
        estimate = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        values = [*estimate, msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                  *msg.pose.covariance]
        if not all(math.isfinite(k) for k in values):
            self.odom_nonfinite += 1
            return
        # Evaluate consistency at 10 Hz, deferred until truth BRACKETS the
        # exact output header timestamp (no future extrapolation).
        if self.odom_count % 10 == 0:
            if len(self.truth_pending) >= self.truth_pending.maxlen:
                self.unmatched_truth += 1
            self.truth_pending.append((s, estimate, tuple(msg.pose.covariance)))
            self.drain_truth_pending()

    def drain_truth_pending(self):
        while self.truth_pending:
            stamp, estimate, covariance = self.truth_pending[0]
            aligned = self.truth.lookup_aligned(stamp, max_bracket_ms=30.)
            if aligned is None:
                # A newer truth may still arrive in the next 10ms tick. Drop
                # only once we can no longer reasonably bracket this stamp.
                if self.truth.samples and self.truth.samples[-1][0] - stamp > 45_000_000:
                    self.truth_pending.popleft()
                    self.unmatched_truth += 1
                    continue
                break
            self.truth_pending.popleft()
            ref = aligned['pose']
            phase = aligned['phase']
            self.truth_bracket_ms.add(aligned['bracket_ms'])
            self.truth_nearest_ms.add(aligned['nearest_ms'])
            nees, eig = nees_pose(estimate, ref, covariance)
            if not math.isfinite(eig) or eig <= 1.e-12:
                self.cov_not_psd += 1
            self.abs_xy_error.add(math.hypot(estimate[0] - ref[0], estimate[1] - ref[1]))
            self.abs_yaw_error.add(abs(wrap(estimate[2] - ref[2])))
            self.per_phase[phase] += 1
            if len(self.calibration_records) < 6500 and self.nees.count % 3 == 0:
                from consistency_metrics import pose_covariance_3x3, pose_error
                self.calibration_records.append({
                    'stamp_ns': int(stamp),
                    'phase': phase, 'cycle': int(aligned['cycle']),
                    'nominal': bool(aligned['valid']),
                    'error': pose_error(estimate, ref).tolist(),
                    'P': pose_covariance_3x3(covariance).tolist(),
                    'bracket_ms': float(aligned['bracket_ms']),
                })
            if nees is None or not math.isfinite(nees):
                continue
            self.nees.add(nees)
            if phase not in self.phase_nees:
                self.phase_nees[phase] = BoundedSamples()
            self.phase_nees[phase].add(nees)
            self.phase_coverage[phase] += int(nees <= CHI2_3_95)
            if aligned['valid']:
                self.nees_normal.add(nees)
                self.covered_nees += int(nees <= CHI2_3_95)
                state_nees = nees_statewise(estimate, ref, covariance)
                if state_nees is not None:
                    for name, value in zip(('x', 'y', 'yaw'), state_nees):
                        self.state_nees_nominal[name].add(value)
                        self.state_coverage_nominal[name] += int(value <= CHI2_1_95)
                cycle = aligned['cycle']
                block = self.cycle_nees.setdefault(cycle, {'n': 0, 'sum_nees': 0., 'covered': 0})
                block['n'] += 1
                block['sum_nees'] += nees
                block['covered'] += int(nees <= CHI2_3_95)

    def on_rt(self, msg):
        p = list(msg.data)
        if len(p) < 14 or not all(math.isfinite(x) for x in p):
            self.odom_nonfinite += 1
            return
        if self.first_profile is None:
            self.first_profile = p
        self.last_profile = p
        self.callback_profiles += 1
        self.callback_sample.add(max(0., p[5]))
        if p[6] > 0:
            self.period_sample.add(p[6])
        seq = int(p[11])
        if seq > 0 and seq != self.last_solve_seq:
            self.solver_sample.add(max(0., p[4]))
            self.last_solve_seq = seq

    def on_worker(self, msg):
        p = list(msg.data)
        if len(p) >= 10:
            self.last_worker = p
            self.worker_seen += 1
            if math.isfinite(p[5]):
                self.cov_worker_age.add(max(0., p[5]))

    def on_nis(self, msg):
        p = list(msg.data)
        if len(p) >= 4:
            for i, v in enumerate(p[:4]):
                if math.isfinite(v) and v >= 0:
                    self.nis[i].add(v)
                    self.nis_count[i] += 1
                    self.nis_greater_95[i] += int(v > CHI2_1_95)

    def on_gating(self, msg):
        self.gating = list(msg.data)

    def on_timing(self, msg):
        self.timing = list(msg.data)

    def publish_sample(self, elapsed, step):
        v, w, phase = motion(elapsed)
        self.last_phase = phase
        now = self.get_clock().now()
        ns = now.nanoseconds
        # Integrate a ground-truth state from the ideal commanded trajectory,
        # NOT from encoder/IMU readings or from the MHE estimated trajectory.
        if self.previous_elapsed is not None:
            dt = min(0.05, max(0., elapsed - self.previous_elapsed))
            v_mid, w_mid, _ = motion(0.5 * (elapsed + self.previous_elapsed))
            heading_mid = self.true_pose[2] + 0.5 * w_mid * dt
            self.true_pose[0] += v_mid * math.cos(heading_mid) * dt
            self.true_pose[1] += v_mid * math.sin(heading_mid) * dt
            self.true_pose[2] = wrap(self.true_pose[2] + w_mid * dt)
        else:
            dt = 0.01
        self.previous_elapsed = elapsed
        a = max(-2., min(2., (v - self.last_true_v) / max(dt, 1e-3)))
        self.last_true_v = v
        # Mark only stable, non-faulted periods as candidates for the nominal
        # NEES calibration statistic. Covariance should also be tested under
        # faults, but that is a separate, non-Gaussian problem.
        phase_age = elapsed % PERIOD
        nominal = phase in ('idle', 'straight', 'turn', 'recovery', 'stop') and \
                  not (phase == 'recovery' and phase_age < 24.) and \
                  not (phase == 'straight' and phase_age < 5.) and \
                  elapsed >= 2.
        self.truth.add(ns, *self.true_pose, valid=nominal,
                       phase=phase, cycle=int(elapsed // PERIOD))
        self.drain_truth_pending()
        # +0.01 rad/s represents an uncalibrated gyro bias; noise seeded.
        if phase != 'imu_dropout':
            imu = Imu()
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = 'base_link'
            imu.orientation.w = 1.
            imu.orientation_covariance[0] = -1.
            imu.angular_velocity.z = w + 0.01 + self.rng.gauss(0, 0.012)
            imu.angular_velocity_covariance[8] = 0.012 ** 2
            imu.linear_acceleration.x = a + self.rng.gauss(0, 0.12)
            imu.linear_acceleration_covariance[0] = 0.12 ** 2
            # Timestamp out-of-order event (at most once per 30-second cycle).
            if (23.5 <= phase_age < 24.0 and
                    self.last_cycle_out_of_order != int(elapsed // PERIOD)):
                self.last_cycle_out_of_order = int(elapsed // PERIOD)
                wrong = ns - 150_000_000
                imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(wrong, 1_000_000_000)
                self.injected['imu_out_of_order'] += 1
            self.pub_imu.publish(imu)
            self.imu_published += 1
        else:
            self.injected['imu_dropout_ticks'] += 1
        if step % 2 == 0:
            if phase != 'wheel_dropout':
                od = Odometry()
                od.header.stamp = now.to_msg()
                od.header.frame_id = 'odom'
                od.child_frame_id = 'base_footprint'
                od.pose.pose.orientation.w = 1.
                # Wheel values are biased by slip but the reference trajectory
                # retains the true kinematics.
                slip = .20 if phase == 'slip' else .0
                od.twist.twist.linear.x = v * (1 + slip) + self.rng.gauss(0, .015)
                od.twist.twist.angular.z = w * (1 + slip) + self.rng.gauss(0, .015)
                if (15.5 <= phase_age < 16.0 and
                    self.last_cycle_spike != int(elapsed // PERIOD)):
                    self.last_cycle_spike = int(elapsed // PERIOD)
                    od.twist.twist.linear.x += 1.2
                    self.injected['encoder_spike'] += 1
                od.twist.covariance[0] = .015 ** 2
                od.twist.covariance[35] = .015 ** 2
                self.pub_wheel.publish(od)
                self.wheel_published += 1
            else:
                self.injected['wheel_dropout_ticks'] += 1

    def report(self, seconds, actual_seconds):
        a = self.first_profile or [0.] * 14
        b = self.last_profile or [0.] * 14
        worker = self.last_worker or [0.] * 10
        callback_miss = max(0., b[8] - a[8])
        solver_miss = max(0., b[7] - a[7])
        solve_count = max(1., b[11] - a[11])
        r = {
            'kind': 'ros_node_synthetic_ground_truth_stress',
            'configured_seconds': seconds, 'elapsed_seconds': actual_seconds,
            'synthetic_ground_truth_only': True,
            'seed': self.seed,
            'injections': dict(self.injected),
            'imu_published': self.imu_published,
            'wheel_published': self.wheel_published,
            'odom_count': self.odom_count,
            'odom_hz': self.odom_count / max(1., actual_seconds),
            'odom_nonfinite_count': self.odom_nonfinite,
            'odom_nonmonotonic_stamp_count': self.odom_nonmonotonic,
            'max_odom_gap_ms': self.max_odom_gap_ms,
            'callback_profiles': self.callback_profiles,
            'solver_ms': self.solver_sample.stats(),
            'callback_ms': self.callback_sample.stats(),
            'callback_period_ms': self.period_sample.stats(),
            'solver_deadline_misses': solver_miss,
            'callback_deadline_misses': callback_miss,
            'solver_deadline_miss_fraction': solver_miss / solve_count,
            'callback_deadline_miss_fraction': callback_miss / max(1., self.callback_profiles),
            'solver_rollbacks': max(0., b[13] - a[13]),
            'covariance_worker': {'submitted': worker[0], 'completed': worker[1],
                'failed': worker[2], 'discarded': worker[4], 'age_solves': worker[5],
                'valid': worker[8], 'observations': self.worker_seen,
                'age_distribution_solves': self.cov_worker_age.stats()},
            'covariance_consistency': {
                'pose_error_nees': self.nees.stats(),
                'nominal_nees': self.nees_normal.stats(),
                'nominal_95pct_coverage': self.covered_nees / max(1, self.nees_normal.count),
                'nominal_95pct_chi2_3_threshold': CHI2_3_95,
                'nominal_state_nees': {k: v.stats() for k, v in self.state_nees_nominal.items()},
                'nominal_state_95pct_coverage': {
                    k: self.state_coverage_nominal[k] / max(1, v.count)
                    for k, v in self.state_nees_nominal.items()},
                'phase_nees': {k: v.stats() for k, v in self.phase_nees.items()},
                'phase_95pct_coverage': {
                    k: self.phase_coverage[k] / max(1, v.count)
                    for k, v in self.phase_nees.items()},
                'independent_cycle_bootstrap': cycle_block_bootstrap(self.cycle_nees),
                'alignment_bracket_ms': self.truth_bracket_ms.stats(),
                'alignment_nearest_ms': self.truth_nearest_ms.stats(),
                'alignment_extrapolation_allowed': False,
                'aligned_pose_records': self.calibration_records,
                'nominal_nees_sample_count': self.nees_normal.count,
                'invalid_or_negative_eigen_count': self.cov_not_psd,
                'truth_unmatched_count': self.unmatched_truth,
                'xy_error_m': self.abs_xy_error.stats(),
                'yaw_error_rad': self.abs_yaw_error.stats(),
                'samples_by_last_published_phase': dict(self.per_phase),
                'caution': 'Synthetic reference; temporally correlated samples and approximate covariance invalidate naive iid chi-square confidence intervals.'},
            'innovation_consistency': {
                'approximate_prefit_nis': [s.stats() for s in self.nis],
                'fraction_above_chi2_1_95': [c / max(1, n) for c, n in zip(self.nis_greater_95, self.nis_count)],
                'channels': ['wheel_left', 'wheel_right', 'gyro_z', 'accel_x'],
                'caution': '/mhe/nis_raw uses approximate model uncertainty, adaptive R and clipping; this is not a validated exact NIS.'},
            'gating_final': self.gating,
            'timing_final': self.timing,
        }
        return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=600.)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--seed', type=int, default=20261008)
    args = ap.parse_args()
    if not (5 <= args.seconds <= 7200):
        ap.error('--seconds must be between 5 and 7200')
    rclpy.init()
    node = StressProbe(seed=args.seed)
    started = time.monotonic()
    next_tick = started
    step = 0
    last_progress = started
    try:
        while time.monotonic() - started < args.seconds:
            now = time.monotonic()
            if now >= next_tick:
                node.publish_sample(now - started, step)
                step += 1
                next_tick += .01
                if now - next_tick > .10:
                    next_tick = now + .01  # avoid burst replay under CPU overload
            rclpy.spin_once(node, timeout_sec=.002)
            if now - last_progress >= 60:
                print(f'STRESS progress: elapsed={now-started:.1f}s, '
                      f'odom={node.odom_count}, solves={node.solver_sample.count}, '
                      f'misses={node.last_profile[8] if node.last_profile else 0}', flush=True)
                last_progress = now
        until = time.monotonic() + .25
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        report = node.report(args.seconds, time.monotonic() - started)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print('STRESS report:', args.output, 'odom:', report['odom_count'],
              'NEES samples:', report['covariance_consistency']['nominal_nees_sample_count'],
              'callback p99:', report['callback_ms']['p99'], flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
