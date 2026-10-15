#!/usr/bin/env python3
"""Smoke test of the running stack, started with `make check`.

Checks that every node is alive by looking at its output: topic rates, latched
topics and the TF chain. It does not move the vehicle.
"""

import sys
import time

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import FluidPressure, Image, Imu, NavSatFix, PointCloud2, Range
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

DURATION = 10.0     # s of wall time

# (topic, type, minimum rate in Hz, container which is publishing it)
# /dvl/twist is not checked here, it comes only with bottom lock and the
# vehicle is at the surface (out of DVL range) when the stack is started.
RATE_CHECKS = [
    ('/clock', Clock, 50.0, 'sim + bridge'),
    ('/ground_truth/odom', Odometry, 30.0, 'sim + bridge'),
    ('/imu/raw', Imu, 30.0, 'sim + bridge'),
    ('/sonar/points', PointCloud2, 3.0, 'sim + bridge'),
    ('/sonar_fan/points', PointCloud2, 6.0, 'sim + bridge'),
    ('/multibeam/swath', PointCloud2, 6.0, 'multibeam'),
    ('/sidescan/waterfall', Image, 1.0, 'sidescan'),
    ('/imu/data', Imu, 30.0, 'ahrs'),
    ('/dvl/altitude', Range, 6.0, 'dvl'),
    ('/gps/fix', NavSatFix, 0.5, 'gps'),
    ('/usbl/pose', PoseWithCovarianceStamped, 0.08, 'usbl'),
    ('/speed_model/twist', TwistWithCovarianceStamped, 6.0, 'controller'),
    ('/depth/pose', PoseWithCovarianceStamped, 15.0, 'depth'),
    ('/pressure', FluidPressure, 15.0, 'depth'),
    ('/propeller/cmd_thrust', Float64, 30.0, 'controller'),
    ('/fins/rudder/cmd_angle', Float64, 30.0, 'controller'),
    ('/fins/elevator/cmd_angle', Float64, 30.0, 'controller'),
    ('/odometry/filtered', Odometry, 30.0, 'localization'),
    ('/odometry/global', Odometry, 20.0, 'navigation'),
    ('/eval/nav_position_error', Float64, 1.0, 'eval'),
    ('/eval/ekf_position_error', Float64, 1.0, 'eval'),
    ('/eval/slam_position_error', Float64, 1.0, 'eval + slam'),
]

# latched topics, one message is enough
LATCHED_CHECKS = [
    ('/usbl/ship', PointStamped, 'usbl'),
    ('/robot_description', String, 'state-pub'),
    ('/mission/status', String, 'mission'),
    ('/mission/path', Path, 'mission'),
]

# (parent, child, container which is publishing it)
TF_CHECKS = [
    ('odom', 'base_link', 'localization'),
    ('map', 'odom', 'navigation'),
    ('slam_map', 'map', 'slam'),
    ('base_link', 'sonar_link', 'state-pub'),
    ('world', 'base_link', 'full chain'),
]


def main():
    rclpy.init()
    node = Node('stack_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    counts = {}

    def counter(topic):
        counts[topic] = 0

        def callback(_):
            counts[topic] += 1
        return callback

    for topic, msg_type, _, _ in RATE_CHECKS:
        node.create_subscription(msg_type, topic, counter(topic), 50)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, msg_type, _ in LATCHED_CHECKS:
        node.create_subscription(msg_type, topic, counter(topic), latched)

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # discovery takes a moment, counting starts after that
    warmup_end = time.time() + 3.0
    while time.time() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for topic, *_ in RATE_CHECKS:
        counts[topic] = 0
    end = time.time() + DURATION
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    failures = 0

    def report(ok, name, detail, source):
        nonlocal failures
        failures += 0 if ok else 1
        print(f'  {"PASS" if ok else "FAIL"}  {name:30s} {detail:24s} [{source}]')

    print(f'\nTopic rates over {DURATION:.0f} s:')
    for topic, _, minimum, source in RATE_CHECKS:
        rate = counts[topic] / DURATION
        report(rate >= minimum, topic, f'{rate:5.1f} Hz (min {minimum:.0f})', source)

    print('\nLatched topics:')
    for topic, _, source in LATCHED_CHECKS:
        report(counts[topic] > 0, topic, 'received' if counts[topic] else 'nothing received', source)

    print('\nTF:')
    for parent, child, source in TF_CHECKS:
        try:
            tf_buffer.lookup_transform(parent, child, Time())
            report(True, f'{parent} -> {child}', 'available', source)
        except TransformException:
            report(False, f'{parent} -> {child}', 'not available', source)

    print(f'\n{"ALL CHECKS PASSED" if failures == 0 else f"{failures} CHECK(S) FAILED"}\n')
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
