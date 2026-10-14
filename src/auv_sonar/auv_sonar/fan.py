"""Reading the across track fan from the simulator."""

import math

import numpy as np
from sensor_msgs_py import point_cloud2

from auv_sonar.acoustics import fan_angles


def declare_fan_parameters(node):
    min_angle = math.radians(node.declare_parameter('fan_min_angle_deg', -85.0).value)
    max_angle = math.radians(node.declare_parameter('fan_max_angle_deg', 85.0).value)
    return min_angle, max_angle


def read_fan(msg, min_angle, max_angle):
    """Hit points (N x 3, inf for no return) and the angle of each ray."""
    raw = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
    points = np.asarray(raw, dtype=float).reshape(-1, 3)
    return points, fan_angles(len(points), min_angle, max_angle)
