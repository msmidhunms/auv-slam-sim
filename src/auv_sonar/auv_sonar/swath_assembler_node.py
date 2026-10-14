"""Puts the multibeam swaths together into bathymetry submaps.

One swath is only a line across the track, so it cannot be matched against
anything. Swaths of `submap_length` metres of track are placed with the
dead reckoning (odom frame), thinned on a voxel grid and published as one
submap in base_link at the time of the last swath. This is the input for SLAM,
the same idea as submap based bathymetric SLAM on real AUVs.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def transform_matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = (t.x, t.y, t.z)
    return m


def apply(matrix, points):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def voxel_thin(points, size):
    """One point per voxel (the first one), keeps the submap small."""
    if len(points) == 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


class SwathAssemblerNode(Node):

    def __init__(self):
        super().__init__('swath_assembler')
        self.submap_length = self.declare_parameter('submap_length', 15.0).value   # m of track
        self.voxel = self.declare_parameter('voxel_size', 0.25).value              # m
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points = []
        self.track = 0.0
        self.last_position = None
        self.last_stamp = None

        self.pending = []           # swaths waiting for the odom transform of their time
        self.max_wait = self.declare_parameter('max_tf_wait', 1.0).value          # s
        self.pub = self.create_publisher(PointCloud2, 'multibeam/submap', 10)
        self.create_subscription(PointCloud2, 'multibeam/swath', self.on_swath, 20)
        self.create_timer(0.05, self.process_pending)

    def on_swath(self, msg):
        # odom TF of the swath time comes a little later than the swath itself,
        # so the swath waits in the queue instead of being dropped
        self.pending.append(msg)

    def process_pending(self):
        while self.pending:
            msg = self.pending[0]
            stamp = Time.from_msg(msg.header.stamp)
            if self.tf_buffer.can_transform(self.odom_frame, msg.header.frame_id, stamp):
                self.pending.pop(0)
                self.add_swath(msg, stamp)
                continue
            newest = Time.from_msg(self.pending[-1].header.stamp)
            if (newest - stamp) > Duration(seconds=self.max_wait) or newest < stamp:
                self.pending.pop(0)
                self.get_logger().warn('no odom transform for a swath, dropped',
                                       throttle_duration_sec=10.0)
                continue
            return

    def add_swath(self, msg, stamp):
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.reset()                       # simulation was reset
        self.last_stamp = stamp
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, msg.header.frame_id, stamp)
        except TransformException:
            return
        swath = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z')).astype(float)
        if len(swath):
            self.points.append(apply(transform_matrix(tf), swath.reshape(-1, 3)))

        position = (tf.transform.translation.x, tf.transform.translation.y)
        if self.last_position is not None:
            self.track += math.dist(position, self.last_position)
        self.last_position = position
        if self.track >= self.submap_length:
            self.publish(msg.header.stamp, stamp)

    def publish(self, stamp_msg, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.odom_frame, stamp)
        except TransformException:
            return
        if self.points:
            cloud = voxel_thin(np.vstack(self.points), self.voxel)
            cloud = apply(transform_matrix(tf), cloud).astype(np.float32)
            header = Header(stamp=stamp_msg, frame_id=self.base_frame)
            self.pub.publish(point_cloud2.create_cloud_xyz32(header, cloud))
        self.reset()

    def reset(self):
        self.points = []
        self.track = 0.0


def main():
    rclpy.init()
    node = SwathAssemblerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
