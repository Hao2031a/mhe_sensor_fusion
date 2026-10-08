#!/usr/bin/env python3
"""Exercise the REAL stress callback logic without needing ROS libraries."""
import importlib
import math
import sys
import types
import unittest
from types import SimpleNamespace as S


def fake(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m

# Stubs represent only ROS APIs used by the stress probe constructor/callback.
rclpy = fake('rclpy')
rclpy.node = fake('rclpy.node')
rclpy.qos = fake('rclpy.qos')
rclpy.qos.qos_profile_sensor_data = object()

class Node:
    def __init__(self, name): self.name = name
    def create_publisher(self, *args): return S(publish=lambda x:None)
    def create_subscription(self, *args): return None
rclpy.node.Node = Node
fake('nav_msgs'); fake('nav_msgs.msg').Odometry = object
fake('sensor_msgs'); fake('sensor_msgs.msg').Imu = object
fake('std_msgs'); fake('std_msgs.msg').Float64MultiArray = object
stress = importlib.import_module('long_stress_test')


def msg(stamp_ns):
    cov=[0.0]*36
    for i in (0,7,35): cov[i]=0.01
    return S(header=S(stamp=S(sec=stamp_ns//10**9,nanosec=stamp_ns%10**9)),
        pose=S(pose=S(position=S(x=0.05,y=0.0),orientation=S(x=0,y=0,z=0,w=1)),covariance=cov),
        twist=S(twist=S(linear=S(x=0.1),angular=S(z=0.0))))


class StressAlignmentTests(unittest.TestCase):
    def test_uses_truth_phase_not_last_callback_phase(self):
        p=stress.StressProbe()
        p.last_phase='stop'
        p.truth.add(1_000_000_000,0,0,0,True,'straight',1)
        p.truth.add(1_010_000_000,.1,0,0,True,'straight',1)
        p.odom_count=9
        p.on_odom(msg(1_005_000_000))
        self.assertEqual(p.per_phase['straight'],1)
        self.assertEqual(p.per_phase['stop'],0)
        self.assertEqual(p.nees_normal.count,1)
        self.assertEqual(p.unmatched_truth,0)
        self.assertEqual(p.cycle_nees[1]['n'],1)

    def test_future_truth_is_waited_for_not_extrapolated(self):
        p=stress.StressProbe()
        p.truth.add(1_000_000_000,0,0,0,True,'turn',0)
        p.odom_count=9
        p.on_odom(msg(1_005_000_000))
        self.assertEqual(len(p.truth_pending),1)
        self.assertEqual(p.nees.count,0)
        p.truth.add(1_010_000_000,.1,0,0,True,'turn',0)
        p.drain_truth_pending()
        self.assertEqual(len(p.truth_pending),0)
        self.assertEqual(p.nees_normal.count,1)
        self.assertLess(p.truth_bracket_ms.stats()['max'], 15)

if __name__=='__main__': unittest.main()
