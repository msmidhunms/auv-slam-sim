"""Small PID helper and angle utilities."""

import math


def wrap_angle(angle):
    """Wrap to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, low, high):
    return max(low, min(high, value))


class Pid:
    """PID with the derivative taken from a measured rate.

    Derivative on measurement is avoiding the kick when the setpoint is
    changed, and here the rate is directly available from gyro and DVL.
    """

    def __init__(self, kp, ki, kd, integral_limit, output_limit, integral_zone=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit    # limit on the integral term output
        self.output_limit = output_limit
        # integral is active only when the error is inside this zone. During a big
        # step it stays off, otherwise it winds up and gives overshoot.
        self.integral_zone = integral_zone
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, rate_error, dt):
        if self.ki > 0.0 and dt > 0.0 and abs(error) < self.integral_zone:
            self.integral += self.ki * error * dt
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        output = self.kp * error + self.integral + self.kd * rate_error
        return clamp(output, -self.output_limit, self.output_limit)
