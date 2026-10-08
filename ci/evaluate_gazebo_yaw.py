#!/usr/bin/env python3
"""Runtime Gazebo A/B test with externally supplied independent truth odometry.
The truth topic must be nav_msgs/msg/Odometry obtained from the simulator,
NOT wheel/filtered odometry; configure or bridge that topic separately.
"""
import argparse
import json
import math
import time
from pathlib import Path

from yaw_metrics import evaluate


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--truth-topic', required=True)
    parser.add_argument('--estimate-topic', default='/odom')
    parser.add_argument('--duration', type=float, default=40.0)
    parser.add_argument('--output', type=Path, default=Path('gazebo_yaw_eval.json'))
    args = parser.parse_args()
    if args.truth_topic == args.estimate_topic:
        parser.error('Ground truth and MHE odom must use different topics')
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('mhe_gazebo_yaw_evaluator')
    truth, estimate = [], []

    def cb_factory(dest):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp > 0:
                dest.append((stamp, yaw_from_quat(msg.pose.pose.orientation)))
        return callback

    subscriptions = [
        node.create_subscription(Odometry, args.truth_topic, cb_factory(truth),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, args.estimate_topic, cb_factory(estimate),
                                 qos_profile_sensor_data),
    ]
    _ = subscriptions
    t0 = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = evaluate(truth, estimate)
    result.update({'truth_topic': args.truth_topic, 'estimate_topic': args.estimate_topic,
                   'real_ros_messages': True, 'gazebo_ground_truth_claim':
                   'requires independent simulator-pose odometry topic supplied by user'})
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
