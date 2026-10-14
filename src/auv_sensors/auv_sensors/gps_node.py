"""GPS model. The antenna is on the top of the vehicle and GPS signals do not go
through water, so a fix is available only at the surface, and only after the
receiver was at the surface for some seconds. Under water the receiver keeps
reporting NO_FIX at 1 Hz.

Outputs:
  gps/fix   sensor_msgs/NavSatFix, latitude / longitude around the datum
  gps/pose  geometry_msgs/PoseWithCovarianceStamped in the map frame, only x and y
            are valid, for the navigation fusion
"""

import random

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus

from auv_sensors.models import enu_to_geodetic, SurfaceFix


class GpsNode(Node):

    def __init__(self):
        super().__init__('gps')
        self.period = 1.0 / self.declare_parameter('rate', 1.0).value
        self.noise_std = self.declare_parameter('noise_std', 2.5).value        # m, horizontal
        self.lat0 = self.declare_parameter('datum_latitude', 36.8023).value
        self.lon0 = self.declare_parameter('datum_longitude', -121.8995).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.frame_id = self.declare_parameter('frame_id', 'gps_link').value
        self.surface = SurfaceFix(self.declare_parameter('surface_z', -0.3).value,
                                  self.declare_parameter('time_to_fix', 10.0).value)
        self.rng = random.Random(self.declare_parameter('seed', 14).value)
        self.last_stamp = None
        self.had_fix = False

        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'gps/pose', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        t = stamp.nanoseconds * 1e-9
        p = msg.pose.pose.position
        has_fix = self.surface.update(p.z, t)     # follows the surfacing at 50 Hz

        if self.last_stamp is not None:
            elapsed = (stamp - self.last_stamp).nanoseconds * 1e-9
            if 0.0 <= elapsed < self.period - 1e-3:
                return
        self.last_stamp = stamp
        if has_fix != self.had_fix:
            self.get_logger().info('GPS fix acquired' if has_fix else 'GPS fix lost, under water')
            self.had_fix = has_fix

        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self.frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS
        if not has_fix:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.fix_pub.publish(fix)
            return

        x = p.x + self.rng.gauss(0.0, self.noise_std)
        y = p.y + self.rng.gauss(0.0, self.noise_std)
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude = enu_to_geodetic(x, y, self.lat0, self.lon0)
        fix.altitude = 0.0
        variance = self.noise_std ** 2
        fix.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, 4 * variance]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = variance
        pose.pose.covariance[7] = variance
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
