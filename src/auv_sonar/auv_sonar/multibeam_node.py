"""Multibeam echosounder: depth points across the track on every ping.

Inner +-60 deg of the simulated fan, with range noise growing towards the outer
beams. Published in the fan frame (x down, y to port), so it can be placed
anywhere with TF.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from auv_sonar.acoustics import multibeam_swath
from auv_sonar.fan import declare_fan_parameters, read_fan


class MultibeamNode(Node):

    def __init__(self):
        super().__init__('multibeam')
        self.fan = declare_fan_parameters(self)
        self.max_angle = math.radians(self.declare_parameter('swath_half_angle_deg', 60.0).value)
        self.range_noise = self.declare_parameter('range_noise', 0.002).value
        self.angle_noise_gain = self.declare_parameter('angle_noise_gain', 3.0).value
        self.rng = np.random.default_rng(self.declare_parameter('seed', 21).value)
        self.pub = self.create_publisher(PointCloud2, 'multibeam/swath', 10)
        self.create_subscription(PointCloud2, 'sonar_fan/points', self.on_fan, 10)

    def on_fan(self, msg):
        points, angles = read_fan(msg, *self.fan)
        swath = multibeam_swath(points, angles, self.max_angle, self.range_noise,
                                self.angle_noise_gain, self.rng)
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, swath.astype(np.float32)))


def main():
    rclpy.init()
    node = MultibeamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
