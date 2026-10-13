import math
import random

from auv_sensors.models import (
    depth_to_pressure,
    GaussMarkov,
    pressure_to_depth,
    quat_from_rpy,
    quat_multiply,
    velocity_at_offset,
    yaw_from_quat,
)
import pytest


def test_velocity_at_offset_pure_translation():
    assert velocity_at_offset((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, -0.1)) == (1.0, 2.0, 3.0)


def test_velocity_at_offset_yaw_rate():
    # point 1 m ahead of the origin, vehicle yawing to the left -> point moves to +y
    v = velocity_at_offset((0.0, 0.0, 0.0), (0.0, 0.0, 0.5), (1.0, 0.0, 0.0))
    assert v == pytest.approx((0.0, 0.5, 0.0))


def test_pressure_depth_round_trip():
    p = depth_to_pressure(10.0, 1000.0, 101325.0)
    assert p == pytest.approx(101325.0 + 98000.0)
    assert pressure_to_depth(p, 1000.0, 101325.0) == pytest.approx(10.0)


def test_surface_pressure_is_atmospheric():
    assert depth_to_pressure(0.0, 1000.0, 101325.0) == pytest.approx(101325.0)


def test_quat_yaw_composition():
    q = quat_multiply(quat_from_rpy(0, 0, 0.3), quat_from_rpy(0, 0, 0.4))
    assert yaw_from_quat(q) == pytest.approx(0.7)


def test_quat_stays_normalised():
    q = quat_multiply(quat_from_rpy(0.1, -0.2, 1.0), quat_from_rpy(0.3, 0.1, -2.0))
    assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0)


def test_gauss_markov_keeps_steady_state_std():
    rng = random.Random(1)
    gm = GaussMarkov(std=2.0, time_constant=10.0, rng=rng)
    samples = [gm.step(1.0) for _ in range(20000)]
    mean = sum(samples) / len(samples)
    std = math.sqrt(sum((s - mean) ** 2 for s in samples) / len(samples))
    assert abs(mean) < 0.2
    assert std == pytest.approx(2.0, rel=0.1)


def test_gauss_markov_disabled():
    gm = GaussMarkov(std=0.0, time_constant=10.0, rng=random.Random(1))
    assert gm.step(1.0) == 0.0
