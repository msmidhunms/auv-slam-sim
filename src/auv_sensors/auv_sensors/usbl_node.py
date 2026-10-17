"""USBL model. A transceiver on the support ship pings the vehicle at a fixed
interval and measures its position from the travel time and the arrival angle
of the reply. The fix is noisy (error grows with slant range), it comes late
(two way sound travel plus processing) and some pings get no reply.

On a real system the fix is sent back to the vehicle by acoustic modem, here it
is published directly with the time stamp of the ping, so the fusion can place
it at the right time.

Outputs:
  usbl/pose  geometry_msgs/PoseWithCovarianceStamped in the map frame, x y z valid
  usbl/ship  geometry_msgs/PointStamped, position of the ship (latched)
"""

import math
import random

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Duration, Time
from std_msgs.msg import Bool

from auv_sensors.models import usbl_latency, usbl_std


class UsblNode(Node):

    def __init__(self):
        super().__init__('usbl')
        self.interval = self.declare_parameter('interval', 5.0).value            # s
        self.ship = tuple(self.declare_parameter('ship_position', [-20.0, -20.0, 0.0]).value)
        self.relative_error = self.declare_parameter('relative_error', 0.005).value
        self.min_error = self.declare_parameter('min_error', 0.2).value          # m
        self.processing = self.declare_parameter('processing_time', 0.4).value   # s
        self.dropout = self.declare_parameter('dropout_probability', 0.1).value
        self.max_range = self.declare_parameter('max_range', 1000.0).value       # m
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.rng = random.Random(self.declare_parameter('seed', 15).value)

        self.next_ping = None
        self.pending = []          # (time when available, message)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'usbl/pose', 10)
        self.ship_pub = self.create_publisher(PointStamped, 'usbl/ship', latched)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)
        self.create_subscription(Bool, 'faults/usbl_off', self.on_fault, 10)
        self.ship_published = False
        self.off = False

    def on_fault(self, msg):
        if msg.data != self.off:
            self.get_logger().warn('fault injected: USBL off' if msg.data else 'USBL back on')
        self.off = msg.data

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if not self.ship_published:
            ship = PointStamped()
            ship.header.stamp = msg.header.stamp
            ship.header.frame_id = self.map_frame
            ship.point.x, ship.point.y, ship.point.z = self.ship
            self.ship_pub.publish(ship)
            self.ship_published = True

        if self.next_ping is None or stamp < self.next_ping - Duration(seconds=self.interval):
            self.next_ping = stamp          # first message, or the simulation was reset
            self.pending.clear()

        # deliver the fixes whose time has come
        while self.pending and self.pending[0][0] <= stamp:
            self.pose_pub.publish(self.pending.pop(0)[1])

        if stamp < self.next_ping:
            return
        self.next_ping = stamp + Duration(seconds=self.interval)

        p = msg.pose.pose.position
        slant = math.dist((p.x, p.y, p.z), self.ship)
        if self.off or slant > self.max_range or self.rng.random() < self.dropout:
            return

        std = usbl_std(slant, self.relative_error, self.min_error)
        fix = PoseWithCovarianceStamped()
        fix.header.stamp = msg.header.stamp          # time of the ping
        fix.header.frame_id = self.map_frame
        fix.pose.pose.position.x = p.x + self.rng.gauss(0.0, std)
        fix.pose.pose.position.y = p.y + self.rng.gauss(0.0, std)
        fix.pose.pose.position.z = p.z + self.rng.gauss(0.0, std)
        fix.pose.pose.orientation.w = 1.0
        for i in (0, 7, 14):
            fix.pose.covariance[i] = std ** 2
        available = stamp + Duration(seconds=usbl_latency(slant, self.processing))
        self.pending.append((available, fix))


def main():
    rclpy.init()
    node = UsblNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
