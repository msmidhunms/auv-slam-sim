"""DVL model.

Bottom lock and altitude come from four Janus beams, 30 deg from the vertical,
which are traced in the simulator (a small gpu_lidar). A beam is valid only if
it hits the seabed within the range of the instrument, and like a real DVL at
least 3 valid beams are needed for bottom lock. So the lock is lost when the
vehicle is too high above the seabed (for example at the surface) or when the
beams miss the seabed.

With lock, body velocity is the simulator ground truth plus noise, scale error
and random ping dropouts. Without lock no velocity is published.
Altitude is published always, as +inf when there is no lock (REP-117).
"""

import math
import random

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from auv_sensors.models import dvl_altitude, velocity_at_offset

# Beam grid of the simulated sensor is 3 x 3 (rows: vertical angle, columns:
# horizontal angle, both -30, 0, +30 deg). The four Janus beams are the middle
# of each edge, corners and centre are not used.
JANUS_BEAMS = ((1, 0), (1, 2), (0, 1), (2, 1))


class DvlNode(Node):

    def __init__(self):
        super().__init__('dvl')
        self.period = 1.0 / self.declare_parameter('rate', 10.0).value
        self.noise_std = self.declare_parameter('noise_std', 0.01).value
        self.dropout = self.declare_parameter('dropout_probability', 0.02).value
        self.frame_id = self.declare_parameter('frame_id', 'dvl_link').value
        self.offset = tuple(self.declare_parameter('offset', [0.3, 0.0, -0.15]).value)
        self.beam_angle = math.radians(self.declare_parameter('beam_angle_deg', 30.0).value)
        self.min_range = self.declare_parameter('min_range', 0.5).value
        self.max_range = self.declare_parameter('max_range', 30.0).value
        self.beam_timeout = self.declare_parameter('beam_timeout', 0.5).value

        self.rng = random.Random(self.declare_parameter('seed', 12).value)
        scale_std = self.declare_parameter('scale_error', 0.002).value
        self.scale = 1.0 + self.rng.gauss(0.0, scale_std)

        self.altitude = None
        self.beams_stamp = None
        self.last_stamp = None
        self.locked = False

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'dvl/twist', 10)
        self.altitude_pub = self.create_publisher(Range, 'dvl/altitude', 10)
        self.failed = False
        self.create_subscription(PointCloud2, 'dvl/beams', self.on_beams, 10)
        self.create_subscription(Bool, 'faults/dvl_failure', self.on_fault, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)

    def on_fault(self, msg):
        if msg.data != self.failed:
            self.get_logger().warn('fault injected: DVL failed' if msg.data else 'DVL fault cleared')
        self.failed = msg.data

    def on_beams(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False))
        width = msg.width
        ranges = []
        for row, col in JANUS_BEAMS:
            index = row * width + col
            if index >= len(points):
                ranges.append(None)
                continue
            x, y, z = (float(c) for c in points[index])
            r = math.sqrt(x * x + y * y + z * z)
            ranges.append(r if math.isfinite(r) and self.min_range <= r <= self.max_range else None)
        self.altitude = dvl_altitude(ranges, self.beam_angle)
        self.beams_stamp = Time.from_msg(msg.header.stamp)

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None:
            elapsed = (stamp - self.last_stamp).nanoseconds * 1e-9
            # small margin so that the output is not dropping to half rate,
            # negative elapsed means the simulation was reset
            if 0.0 <= elapsed < self.period - 1e-3:
                return
        self.last_stamp = stamp

        altitude = self.altitude
        if self.failed or self.beams_stamp is None or \
                (stamp - self.beams_stamp).nanoseconds * 1e-9 > self.beam_timeout:
            altitude = None
        self.publish_altitude(msg.header.stamp, altitude)

        locked = altitude is not None
        if locked != self.locked:
            self.get_logger().info('bottom lock acquired' if locked else 'bottom lock lost')
            self.locked = locked
        if not locked or self.rng.random() < self.dropout:
            return

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
        self.twist_pub.publish(out)

    def publish_altitude(self, stamp, altitude):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 2 * self.beam_angle
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = altitude if altitude is not None else math.inf
        self.altitude_pub.publish(msg)


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
