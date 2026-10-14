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


# --- DVL, GPS, USBL --------------------------------------------------------------

from auv_sensors.models import (  # noqa: E402
    dvl_altitude,
    enu_to_geodetic,
    geodetic_to_enu,
    SurfaceFix,
    usbl_latency,
    usbl_std,
)


def test_dvl_altitude_from_four_beams():
    beam = math.radians(30)
    ranges = [10.0 / math.cos(beam)] * 4          # flat seabed 10 m below
    assert dvl_altitude(ranges, beam) == pytest.approx(10.0)


def test_dvl_lock_needs_three_beams():
    beam = math.radians(30)
    assert dvl_altitude([10.0, 10.0, 10.0, None], beam) is not None
    assert dvl_altitude([10.0, None, 10.0, None], beam) is None
    assert dvl_altitude([math.inf, math.inf, 10.0, 10.0], beam) is None


def test_geodetic_round_trip():
    lat, lon = enu_to_geodetic(120.0, -340.0, 36.8, -121.9)
    x, y = geodetic_to_enu(lat, lon, 36.8, -121.9)
    assert x == pytest.approx(120.0, abs=1e-3)
    assert y == pytest.approx(-340.0, abs=1e-3)


def test_one_degree_latitude_is_around_111_km():
    lat, _ = enu_to_geodetic(0.0, 111319.5, 0.0, 0.0)
    assert lat == pytest.approx(1.0, abs=1e-4)


def test_gps_fix_only_after_some_time_at_surface():
    s = SurfaceFix(surface_z=-0.3, time_to_fix=10.0)
    assert not s.update(-5.0, 0.0)               # under water
    assert not s.update(-0.1, 1.0)               # just surfaced
    assert not s.update(-0.1, 10.9)
    assert s.update(-0.1, 11.0)                  # 10 s at the surface
    assert not s.update(-1.0, 12.0)              # dived again
    assert not s.update(-0.1, 13.0)              # timer starts again


def test_usbl_error_grows_with_range():
    assert usbl_std(10.0, 0.005, 0.2) == pytest.approx(0.2)       # minimum
    assert usbl_std(200.0, 0.005, 0.2) == pytest.approx(1.0)


def test_usbl_latency():
    assert usbl_latency(150.0, 0.4) == pytest.approx(0.6)
