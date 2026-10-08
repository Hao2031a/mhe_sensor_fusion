#!/usr/bin/env python3
"""ROS 2 integration smoke: drive the actual Ceres MHE node, not a Python surrogate.
Publishes time-stamped IMU (100 Hz) + wheel odometry (50 Hz) in wall/ROS time.
Validates finite/monotonic /odom output, covariance, diagnostics, and liveness.

Do not interpret this as robot-ground-truth accuracy, real-time WCET, or a Gazebo test.
"""
import argparse
import json
import math
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__('mhe_ci_probe')
        self.pub_wheel = self.create_publisher(Odometry, '/odom/unfiltered', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.odom = []
        self.output_timing = []
        self.solver = []
        self.gating = []
        self.solve_times = []
        self.rt_profiles = []
        self.cov_worker = []
        self.time_alignment = []
        self.graph_status = []
        self.output_status = []
        self.create_subscription(Float64MultiArray, '/mhe/output_status',
                                 lambda m: self.output_status.append(list(m.data)), 100)
        self.create_subscription(Float64MultiArray, "/mhe/graph_status",
                                 lambda m: self.graph_status.append(list(m.data)), 100)
        self.create_subscription(Float64MultiArray, '/mhe/time_alignment',
                                 lambda m: self.time_alignment.append(list(m.data)), 100)
        self.create_subscription(Float64MultiArray, '/mhe/cov_worker_status',
                                 lambda m: self.cov_worker.append(list(m.data)), 100)
        self.create_subscription(Odometry, '/mhe_ci/odom', self.on_odom, 100)
        self.create_subscription(Float64MultiArray, '/mhe/timing', lambda m: self.output_timing.append(list(m.data)), 100)
        self.create_subscription(Float64MultiArray, '/mhe/solver_health', lambda m: self.solver.append(list(m.data)), 100)
        self.create_subscription(Float64MultiArray, '/mhe/gating_status', lambda m: self.gating.append(list(m.data)), 100)
        self.create_subscription(Float64, '/mhe/solve_time_ms', lambda m: self.solve_times.append(float(m.data)), 100)
        self.create_subscription(Float64MultiArray, '/mhe/rt_profile',
                                 lambda m: self.rt_profiles.append(list(m.data)), 100)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist
        fields = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  v.linear.x, v.angular.z, *msg.pose.covariance, *msg.twist.covariance]
        self.odom.append({'stamp_ns': stamp_ns(msg), 'fields_finite': all(math.isfinite(x) for x in fields),
                          'x': p.x, 'y': p.y, 'v': v.linear.x, 'w': v.angular.z,
                          'variance_x': msg.pose.covariance[0],
                          'variance_y': msg.pose.covariance[7],
                          'variance_yaw': msg.pose.covariance[35]})

    def publish_sample(self, elapsed, step, inject_spike=False):
        # Smooth acceleration and return to zero. Small angular motion after 3s.
        if elapsed < 1.0:
            v, w = 0.0, 0.0
        elif elapsed < 1.5:
            v, w = 0.18 * (elapsed - 1.0) / 0.5, 0.0
        elif elapsed < 3.0:
            v, w = 0.18, 0.0
        elif elapsed < 3.7:
            v, w = 0.12, 0.30
        elif elapsed < 4.6:
            v, w = 0.12, 0.0
        else:
            v, w = 0.0, 0.0
        a = 0.36 if 1.0 <= elapsed < 1.5 else 0.0
        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0  # no trusted orientation
        imu.angular_velocity.z = w
        imu.linear_acceleration.x = a
        self.pub_imu.publish(imu)

        if step % 2 == 0:
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = 'odom'
            od.child_frame_id = 'base_footprint'
            od.pose.pose.orientation.w = 1.0
            od.twist.twist.linear.x = 1.2 if inject_spike else v
            od.twist.twist.angular.z = w
            od.twist.covariance[0] = 0.0025
            od.twist.covariance[35] = 0.0025
            self.pub_wheel.publish(od)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    rclpy.init()
    probe = Probe()
    started = time.monotonic()
    seconds = float(os.getenv('MHE_CI_SMOKE_SECONDS', '7.0'))
    next_tick = started
    step = 0
    spike_done = False
    try:
        while time.monotonic() - started < seconds:
            current = time.monotonic()
            if current >= next_tick:
                elapsed = current - started
                inject = 3.9 <= elapsed < 4.1 and not spike_done
                probe.publish_sample(elapsed, step, inject_spike=inject)
                if inject and step % 2 == 0:
                    spike_done = True
                step += 1
                next_tick += 0.01
                if current - next_tick > 0.1:
                    next_tick = current + 0.01
            rclpy.spin_once(probe, timeout_sec=0.002)
        # Drain late results without publishing: output node may stop after timeout.
        until = time.monotonic() + 0.2
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=0.005)
        records = probe.odom
        stamps = [x['stamp_ns'] for x in records]
        times_ms = [x for x in probe.solve_times if math.isfinite(x) and x > 0]
        times_ms.sort()
        percentile = lambda a, frac: a[min(len(a)-1, round((len(a)-1)*frac))] if a else None
        # One profile per output tick; deduplicate by monotonic solve sequence.
        profiles = [p for p in probe.rt_profiles if len(p) >= 14]
        unique_solves = {}
        for profile in profiles:
            if profile[11] > 0:
                unique_solves[int(profile[11])] = profile
        solve_profiles = list(unique_solves.values())
        def latency_stats(values):
            a = sorted(v for v in values if math.isfinite(v) and v >= 0)
            return {'count': len(a), 'p50': percentile(a, .50),
                    'p95': percentile(a, .95), 'p99': percentile(a, .99),
                    'max': max(a) if a else None}
        solver_total = latency_stats(p[4] for p in solve_profiles)
        ceres_only = latency_stats(p[1] for p in solve_profiles)
        graph_build = latency_stats(p[0] for p in solve_profiles)
        covariance = latency_stats(p[2] for p in solve_profiles)
        workers = [p for p in probe.cov_worker if len(p) >= 10]
        # Deduplicate the same background result reported by several 100Hz ticks.
        worker_unique = {}
        for q in workers:
            if q[9] > 0 and q[8] == 1.0:
                worker_unique[int(q[9])] = q[6]
        worker_stats = latency_stats(worker_unique.values())
        marginalization = latency_stats(p[3] for p in solve_profiles)
        callback_wall = latency_stats(p[5] for p in profiles)
        interval_wall = latency_stats(p[6] for p in profiles if p[6] > 0)
        end = profiles[-1] if profiles else [0] * 14
        begin = profiles[0] if profiles else [0] * 14
        solve_count = max(1.0, end[11] - begin[11])
        callback_count = max(1, len(profiles))
        rt_metrics = {
            'graph_build_ms': graph_build,
            'ceres_only_ms': ceres_only,
            'covariance_ms': covariance,
            'covariance_snapshot_ms': covariance,
            'covariance_worker_last': workers[-1] if workers else [],
            'covariance_worker_compute_ms': workers[-1][6] if workers else None,
            'covariance_worker_compute_stats_ms': worker_stats,
            'covariance_worker_submitted': workers[-1][0] if workers else 0,
            'covariance_worker_completed': workers[-1][1] if workers else 0,
            'covariance_worker_failed': workers[-1][2] if workers else 0,
            'covariance_worker_discarded': workers[-1][4] if workers else 0,
            'covariance_worker_age_solves': workers[-1][5] if workers else None,
            'covariance_worker_valid': workers[-1][8] if workers else 0,
            'marginalization_ms': marginalization,
            'total_solve_ms': solver_total,
            'callback_wall_ms': callback_wall,
            'callback_period_ms': interval_wall,
            'solve_deadline_miss_count': end[7] - begin[7],
            'callback_deadline_miss_count': end[8] - begin[8],
            'covariance_deferred_count': end[9] - begin[9],
            'covariance_forced_count': end[10] - begin[10],
            'late_tick_count': end[12] - begin[12],
            'rollback_count': end[13] - begin[13],
            'solve_deadline_miss_fraction': max(0, end[7] - begin[7]) / solve_count,
            'callback_deadline_miss_fraction': max(0, end[8] - begin[8]) / callback_count,
            'last_profile': end,
            'profile_format': 'build,ceres,cov,marg,total,callback,period,solve_miss,callback_miss,cov_deferred,cov_forced,solve_sequence,tick_late,rollbacks',
        }
        checks = {
            'odom_messages_ge_80': len(records) >= 80,
            'all_output_finite': all(x['fields_finite'] for x in records),
            'output_stamps_strictly_increasing': all(b > a for a, b in zip(stamps, stamps[1:])),
            'nonnegative_xy_yaw_covariance': all(x['variance_x'] >= 0 and x['variance_y'] >= 0 and x['variance_yaw'] >= 0 for x in records),
            'solver_health_published': len(probe.solver) > 10,
            'solver_health_extended_schema': len(probe.solver) > 10 and all(
                len(x) >= 14 and all(math.isfinite(v) for v in x[:14]) and
                int(x[11]) in (0, 1, 2, 3) and x[12] >= 0 and x[13] >= 0
                for x in probe.solver),
            'timing_published': len(probe.output_timing) > 10,
            'timestamp_alignment_published': len(probe.time_alignment) > 10,
            'timestamp_alignment_schema_valid': bool(probe.time_alignment) and all(
                len(t) >= 11 and all(math.isfinite(v) for v in t[:11]) and
                int(t[3]) in range(7) and t[1] >= 0 and
                t[1] <= 0.050001 for t in probe.time_alignment),
            'gating_published': len(probe.gating) > 10,
            'solver_executed': bool(times_ms),
            'bounded_output': all(abs(x['v']) <= 1.6 and abs(x['w']) <= 6.1 for x in records),
            'graph_status_published': len(probe.graph_status) >= 80,
            'graph_status_schema_valid': bool(probe.graph_status) and all(
                len(q) >= 11 and all(math.isfinite(v) for v in q[:11]) and
                q[6] >= 0 and q[7] >= 0 and q[2] >= 0 and q[4] >= 0
                for q in probe.graph_status),
            'rt_profile_published': len(profiles) >= 80,
            'rt_profile_finite': all(all(math.isfinite(v) for v in p) for p in profiles),
            'rt_solver_profile_nonnegative': len(solve_profiles) >= 20 and all(
                all(p[i] >= 0.0 for i in range(5)) for p in solve_profiles),
            'covariance_worker_published': len(workers) >= 80,
            'covariance_worker_completed': bool(workers) and workers[-1][1] >= 2,
            'covariance_worker_result_valid': bool(workers) and workers[-1][8] == 1.0,
            'covariance_worker_finite': all(all(math.isfinite(v) for v in q) for q in workers),
        }
        result = {'pass': all(checks.values()), 'checks': checks, 'published_samples': step,
                  'odom_samples': len(records), 'timing_samples': len(probe.output_timing),
                  'solver_health_samples': len(probe.solver),
                  'injected_encoder_spike': spike_done,
                  'last_solver_health': probe.solver[-1] if probe.solver else [],
                  'solver_fallback_diagnostics': {
                      'rejection_reason': int(probe.solver[-1][11]) if probe.solver and len(probe.solver[-1]) >= 14 else None,
                      'qr_retry_total': int(probe.solver[-1][12]) if probe.solver and len(probe.solver[-1]) >= 14 else None,
                      'qr_skip_total': int(probe.solver[-1][13]) if probe.solver and len(probe.solver[-1]) >= 14 else None,
                  },
                  'last_graph_status': probe.graph_status[-1] if probe.graph_status else [],
                  'last_covariance_worker': probe.cov_worker[-1] if probe.cov_worker else [],
                  'last_output_status': probe.output_status[-1] if probe.output_status else [],
                  'last_gating': probe.gating[-1] if probe.gating else [],
                  'last_timing': probe.output_timing[-1] if probe.output_timing else [],
                  'last_time_alignment': probe.time_alignment[-1] if probe.time_alignment else [],
                  'solver_time_ms': {'count': len(times_ms), 'p50': percentile(times_ms,.50),
                                     'p95': percentile(times_ms,.95), 'p99': percentile(times_ms,.99),
                                     'max': max(times_ms) if times_ms else None},
                  'realtime_profile': rt_metrics}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n',encoding='utf-8')
        print(json.dumps(result, indent=2))
        if not result['pass']:
            raise SystemExit(1)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
