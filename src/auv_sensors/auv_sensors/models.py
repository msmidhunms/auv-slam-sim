"""Sensor maths, kept free from ROS so that it can be unit tested directly."""

import math

GRAVITY = 9.8


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def velocity_at_offset(linear, angular, offset):
    """Body frame velocity of a point which is at `offset` from the body origin."""
    w_x_r = cross(angular, offset)
    return tuple(v + c for v, c in zip(linear, w_x_r))


def depth_to_pressure(depth, density, atmospheric):
    """Absolute pressure in Pa at the given depth in m (positive down)."""
    return atmospheric + density * GRAVITY * depth


def pressure_to_depth(pressure, density, atmospheric):
    return (pressure - atmospheric) / (density * GRAVITY)


def quat_multiply(a, b):
    """Hamilton product, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class GaussMarkov:
    """First order Gauss-Markov process, used for slowly wandering sensor bias."""

    def __init__(self, std, time_constant, rng):
        self.std = std
        self.tau = time_constant
        self.rng = rng
        self.value = rng.gauss(0.0, std) if std > 0 else 0.0

    def step(self, dt):
        if self.std <= 0 or dt <= 0:
            return self.value
        a = math.exp(-dt / self.tau)
        # driving noise is scaled so that the steady state std stays at self.std
        self.value = a * self.value + self.std * math.sqrt(1 - a * a) * self.rng.gauss(0.0, 1.0)
        return self.value
