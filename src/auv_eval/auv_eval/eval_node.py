"""Compares the EKF and SLAM estimates with the simulator ground truth.

Errors are published as plain numbers so they can be plotted live in Foxglove,
and a summary is logged at regular interval. This node is only for simulation,
on the real vehicle there is no ground truth.

World frame of the simulator and the map frame are taken as the same, which
holds when the simulator and localisation are started together.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RunningStats:

    def __init__(self):
        self.count = 0
        self.sum_squares = 0.0
        self.maximum = 0.0

    def add(self, value):
        self.count += 1
        self.sum_squares += value * value
        self.maximum = max(self.maximum, abs(value))

    @property
    def rms(self):
        return math.sqrt(self.sum_squares / self.count) if self.count else 0.0


class EvalNode(Node):

    def __init__(self):
        super().__init__('eval')
        rate = self.declare_parameter('rate', 2.0).value
        self.summary_period = self.declare_parameter('summary_period', 60.0).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.truth = None
        self.ekf = None
        self.last_truth_xy = None
        self.reset()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {name: self.create_publisher(Float64, f'eval/{name}', 10) for name in (
            'ekf_position_error', 'slam_position_error', 'depth_error',
            'heading_error_deg', 'distance_travelled')}
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_truth, 10)
        self.create_subscription(Odometry, 'odometry/filtered', self.on_ekf, 10)
        self.create_service(Trigger, 'eval/reset', self.on_reset)
        self.create_timer(1.0 / rate, self.on_timer)
        self.create_timer(self.summary_period, self.on_summary)

    def reset(self):
        self.distance = 0.0
        self.ekf_stats = RunningStats()
        self.slam_stats = RunningStats()

    def on_reset(self, request, response):
        self.reset()
        response.success = True
        response.message = 'statistics cleared'
        return response

    def on_truth(self, msg):
        self.truth = msg
        p = msg.pose.pose.position
        if self.last_truth_xy is not None:
            self.distance += math.hypot(p.x - self.last_truth_xy[0], p.y - self.last_truth_xy[1])
        self.last_truth_xy = (p.x, p.y)

    def on_ekf(self, msg):
        self.ekf = msg

    def on_timer(self):
        if self.truth is None or self.ekf is None:
            return
        t = self.truth.pose.pose
        e = self.ekf.pose.pose

        ekf_error = math.hypot(e.position.x - t.position.x, e.position.y - t.position.y)
        self.ekf_stats.add(ekf_error)
        self.publish('ekf_position_error', ekf_error)
        self.publish('depth_error', e.position.z - t.position.z)
        self.publish('heading_error_deg', math.degrees(
            wrap_angle(yaw_from_quat(e.orientation) - yaw_from_quat(t.orientation))))
        self.publish('distance_travelled', self.distance)

        # SLAM pose is map -> base_link, it is not there till the slam node is up
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return
        s = tf.transform.translation
        slam_error = math.hypot(s.x - t.position.x, s.y - t.position.y)
        self.slam_stats.add(slam_error)
        self.publish('slam_position_error', slam_error)

    def on_summary(self):
        if self.ekf_stats.count == 0:
            return
        self.get_logger().info(
            f'{self.distance:.0f} m travelled | position error: '
            f'EKF rms {self.ekf_stats.rms:.2f} max {self.ekf_stats.maximum:.2f} m, '
            f'SLAM rms {self.slam_stats.rms:.2f} max {self.slam_stats.maximum:.2f} m')

    def publish(self, name, value):
        self.pubs[name].publish(Float64(data=float(value)))


def main():
    rclpy.init()
    node = EvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
