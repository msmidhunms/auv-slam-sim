import math

from auv_control.pid import clamp, Pid, wrap_angle
import pytest


def test_wrap_angle():
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap_angle(5 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_heading_error_takes_short_way():
    # from 170 deg to -170 deg is 20 deg, not 340 deg
    error = wrap_angle(math.radians(-170) - math.radians(170))
    assert error == pytest.approx(math.radians(20))


def test_clamp():
    assert clamp(5, -1, 1) == 1
    assert clamp(-5, -1, 1) == -1
    assert clamp(0.3, -1, 1) == 0.3


def test_proportional_and_derivative():
    pid = Pid(kp=2.0, ki=0.0, kd=0.5, integral_limit=0.0, output_limit=100.0)
    assert pid.update(error=1.0, rate_error=-2.0, dt=0.02) == pytest.approx(2.0 - 1.0)


def test_integral_accumulates_and_is_limited():
    pid = Pid(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5, output_limit=100.0)
    for _ in range(10):
        out = pid.update(error=1.0, rate_error=0.0, dt=0.02)
    assert out == pytest.approx(0.2)
    for _ in range(1000):
        out = pid.update(error=1.0, rate_error=0.0, dt=0.02)
    assert out == pytest.approx(0.5)


def test_output_limit_and_reset():
    pid = Pid(kp=10.0, ki=1.0, kd=0.0, integral_limit=5.0, output_limit=3.0)
    assert pid.update(error=1.0, rate_error=0.0, dt=0.1) == 3.0
    pid.reset()
    assert pid.integral == 0.0


def test_integral_zone():
    pid = Pid(kp=0.0, ki=1.0, kd=0.0, integral_limit=5.0, output_limit=100.0, integral_zone=0.3)
    pid.update(error=2.0, rate_error=0.0, dt=1.0)       # outside the zone, no integration
    assert pid.integral == 0.0
    pid.update(error=0.2, rate_error=0.0, dt=1.0)
    assert pid.integral == pytest.approx(0.2)
