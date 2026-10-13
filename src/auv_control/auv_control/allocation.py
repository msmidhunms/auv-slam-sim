"""Thruster allocation maths, free from ROS so that it can be unit tested."""

import math

import numpy as np


def rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def allocation_matrix(thrusters):
    """6 x N matrix which maps the thruster forces to the body wrench.

    Each column is [direction, position x direction] of one thruster, thrust is
    acting along the +x axis of the thruster frame.
    """
    columns = []
    for t in thrusters:
        position = np.array(t['xyz'], dtype=float)
        direction = rotation_from_rpy(*t['rpy']) @ np.array([1.0, 0.0, 0.0])
        columns.append(np.concatenate([direction, np.cross(position, direction)]))
    return np.array(columns).T


class ThrusterAllocator:

    def __init__(self, thrusters, max_thrust):
        self.matrix = allocation_matrix(thrusters)
        if np.linalg.matrix_rank(self.matrix) < 6:
            raise ValueError('thruster layout is not giving full 6 DOF control')
        # minimum norm solution, it shares the load across the thrusters
        self.inverse = np.linalg.pinv(self.matrix)
        self.max_thrust = float(max_thrust)

    def allocate(self, wrench):
        """Thruster forces for the body wrench [fx, fy, fz, tx, ty, tz]."""
        thrusts = self.inverse @ np.asarray(wrench, dtype=float)
        peak = np.max(np.abs(thrusts))
        if peak > self.max_thrust:
            # scale all together, so the direction of the wrench is not changing
            thrusts *= self.max_thrust / peak
        return thrusts

    def wrench(self, thrusts):
        return self.matrix @ np.asarray(thrusts, dtype=float)
