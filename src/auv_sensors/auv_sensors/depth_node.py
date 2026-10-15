"""Pressure sensor model. Publishes the absolute pressure like the real sensor
does, and the depth computed back from it as a pose for the EKF."""

import random

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import FluidPressure

from auv_sensors.models import GRAVITY, depth_to_pressure, pressure_to_depth


class DepthNode(Node):

    def __init__(self):
        super().__init__('depth')
        self.period = 1.0 / self.declare_parameter('rate', 20.0).value
        self.noise_std = self.declare_parameter('noise_std', 0.01).value
        # depth is the same in every world fixed frame, it is published once per
        # frame so that each EKF gets it in its own world frame. Otherwise the
        # global EKF has to transform it with the map -> odom TF which it is
        # publishing itself, and that feedback made it diverge (tested).
        self.world_frames = self.declare_parameter('world_frames', ['odom', 'map']).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.density = self.declare_parameter('water_density', 1000.0).value
        self.atmospheric = self.declare_parameter('atmospheric_pressure', 101325.0).value

        self.rng = random.Random(self.declare_parameter('seed', 13).value)
        self.pressure_std = self.noise_std * self.density * GRAVITY
        self.last_stamp = None

        self.pressure_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.depth_pubs = {
            frame: self.create_publisher(
                PoseWithCovarianceStamped, 'depth/pose' if i == 0 else f'depth/pose_{frame}', 10)
            for i, frame in enumerate(self.world_frames)}
        self.sub = self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None:
            elapsed = (stamp - self.last_stamp).nanoseconds * 1e-9
            if 0.0 <= elapsed < self.period - 1e-3:
                return
        self.last_stamp = stamp

        true_depth = -msg.pose.pose.position.z
        pressure = depth_to_pressure(true_depth, self.density, self.atmospheric)
        pressure += self.rng.gauss(0.0, self.pressure_std)

        p = FluidPressure()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = self.frame_id
        p.fluid_pressure = pressure
        p.variance = self.pressure_std ** 2
        self.pressure_pub.publish(p)

        # only z is valid here, EKF config is taking z alone from this topic
        z = -pressure_to_depth(pressure, self.density, self.atmospheric)
        for frame, pub in self.depth_pubs.items():
            d = PoseWithCovarianceStamped()
            d.header.stamp = msg.header.stamp
            d.header.frame_id = frame
            d.pose.pose.position.z = z
            d.pose.pose.orientation.w = 1.0
            d.pose.covariance[14] = self.noise_std ** 2
            pub.publish(d)


def main():
    rclpy.init()
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
