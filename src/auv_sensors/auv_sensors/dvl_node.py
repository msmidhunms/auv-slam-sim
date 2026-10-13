"""DVL model. Body frame velocity from the simulator ground truth with noise,
scale error and bottom lock dropouts."""

import random

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from auv_sensors.models import velocity_at_offset


class DvlNode(Node):

    def __init__(self):
        super().__init__('dvl')
        self.period = 1.0 / self.declare_parameter('rate', 10.0).value
        self.noise_std = self.declare_parameter('noise_std', 0.01).value
        self.dropout = self.declare_parameter('dropout_probability', 0.02).value
        self.frame_id = self.declare_parameter('frame_id', 'dvl_link').value
        self.offset = tuple(self.declare_parameter('offset', [0.0, 0.0, -0.10]).value)

        self.rng = random.Random(self.declare_parameter('seed', 12).value)
        # scale error is fixed for one instrument, so drawn only once
        scale_std = self.declare_parameter('scale_error', 0.002).value
        self.scale = 1.0 + self.rng.gauss(0.0, scale_std)
        self.last_stamp = None

        self.pub = self.create_publisher(TwistWithCovarianceStamped, 'dvl/twist', 10)
        self.sub = self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None:
            elapsed = (stamp - self.last_stamp).nanoseconds * 1e-9
            # small margin so that the output is not dropping to half rate
            # negative elapsed means the simulation was reset
            if 0.0 <= elapsed < self.period - 1e-3:
                return
        self.last_stamp = stamp

        if self.rng.random() < self.dropout:
            return      # no bottom lock for this ping

        twist = msg.twist.twist
        linear = (twist.linear.x, twist.linear.y, twist.linear.z)
        angular = (twist.angular.x, twist.angular.y, twist.angular.z)
        velocity = velocity_at_offset(linear, angular, self.offset)

        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.twist.twist.linear.x = self.scale * velocity[0] + self.rng.gauss(0.0, self.noise_std)
        out.twist.twist.linear.y = self.scale * velocity[1] + self.rng.gauss(0.0, self.noise_std)
        out.twist.twist.linear.z = self.scale * velocity[2] + self.rng.gauss(0.0, self.noise_std)
        for i in (0, 7, 14):
            out.twist.covariance[i] = self.noise_std ** 2
        # angular velocity is not measured by a DVL
        for i in (21, 28, 35):
            out.twist.covariance[i] = -1.0

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DvlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
