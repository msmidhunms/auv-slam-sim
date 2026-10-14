"""Sidescan sonar: echo intensity against slant range on both sides, shown as
a scrolling waterfall image like on the sidescan display of a survey software.

Port is on the left, starboard on the right, the newest ping is on the top. The
dark band in the middle is the water column under the vehicle, dark areas
behind objects are acoustic shadows. Image is slant range (not corrected to
ground range), which is what the raw sidescan record looks like.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from auv_sonar.acoustics import sidescan_ping, waterfall_row
from auv_sonar.fan import declare_fan_parameters, read_fan


def copper(values):
    """Grey levels 0..1 to the brown / gold colour of sidescan displays."""
    v = np.clip(values, 0.0, 1.0) ** 0.7
    return np.stack([np.minimum(1.0, 1.25 * v), 0.78 * v, 0.50 * v], axis=-1)


class SidescanNode(Node):

    def __init__(self):
        super().__init__('sidescan')
        self.fan = declare_fan_parameters(self)
        self.max_range = self.declare_parameter('max_range', 60.0).value       # m per side
        self.bins = self.declare_parameter('bins', 300).value                  # per side
        self.rows = self.declare_parameter('waterfall_rows', 400).value
        self.publish_every = self.declare_parameter('publish_every', 5).value  # pings
        self.gain = self.declare_parameter('gain', 1.6).value
        self.rng = np.random.default_rng(self.declare_parameter('seed', 22).value)

        self.waterfall = np.zeros((self.rows, 2 * self.bins))
        self.pings = 0
        self.pub = self.create_publisher(Image, 'sidescan/waterfall', 10)
        self.create_subscription(PointCloud2, 'sonar_fan/points', self.on_fan, 10)

    def on_fan(self, msg):
        points, angles = read_fan(msg, *self.fan)
        port, starboard = sidescan_ping(points, angles, self.max_range, self.bins, self.rng)
        self.waterfall = np.roll(self.waterfall, 1, axis=0)
        self.waterfall[0] = waterfall_row(port, starboard) * self.gain
        self.pings += 1
        if self.pings % self.publish_every == 0:
            self.publish(msg.header)

    def publish(self, header):
        rgb = (copper(self.waterfall) * 255).astype(np.uint8)
        img = Image()
        img.header.stamp = header.stamp
        img.header.frame_id = header.frame_id
        img.height, img.width = rgb.shape[:2]
        img.encoding = 'rgb8'
        img.step = img.width * 3
        img.data = rgb.tobytes()
        self.pub.publish(img)


def main():
    rclpy.init()
    node = SidescanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
