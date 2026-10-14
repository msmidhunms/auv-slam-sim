import math

from auv_control.torpedo import (
    altitude_to_depth,
    default_params,
    drag_thrust,
    gain_schedule,
    Refs,
    State,
    TorpedoController,
)
import pytest

DT = 0.02


def level_state(**kw):
    values = dict(u=1.0, z=-10.0, w_world=0.0, pitch=0.0, yaw=0.0, q=0.0, r=0.0)
    values.update(kw)
    return State(**values)


def test_gain_schedule():
    assert gain_schedule(1.0, 1.0, 0.4, 4.0) == pytest.approx(1.0)
    assert gain_schedule(2.0, 1.0, 0.4, 4.0) == pytest.approx(0.25)
    assert gain_schedule(0.1, 1.0, 0.4, 4.0) == pytest.approx(4.0)     # clipped


def test_drag_thrust_matches_hull_drag():
    assert drag_thrust(1.0, 1.0, 6.2282) == pytest.approx(7.2282)
    assert drag_thrust(0.0, 1.0, 6.2282) == 0.0


def test_altitude_to_depth():
    # 15 m above the seabed, want 10 m -> go 5 m deeper
    assert altitude_to_depth(z=-20.0, altitude=15.0, altitude_ref=10.0) == pytest.approx(-25.0)


def test_steady_state_is_quiet():
    c = TorpedoController(default_params())
    out = c.update(level_state(), Refs(speed=1.0, yaw=0.0, z=-10.0), DT)
    assert out.thrust == pytest.approx(7.2282, abs=1e-3)
    assert out.rudder == pytest.approx(0.0)
    assert out.elevator == pytest.approx(0.0)


def test_turn_right_gives_positive_rudder():
    c = TorpedoController(default_params())
    out = c.update(level_state(), Refs(speed=1.0, yaw=math.radians(-30), z=-10.0), DT)
    assert out.rudder > 0.0


def test_heading_error_wraps():
    c = TorpedoController(default_params())
    # at 170 deg, target -170 deg is 20 deg to the left -> negative rudder
    out = c.update(level_state(yaw=math.radians(170)),
                   Refs(speed=1.0, yaw=math.radians(-170), z=-10.0), DT)
    assert out.rudder < 0.0


def test_go_deeper_gives_nose_down():
    c = TorpedoController(default_params())
    out = c.update(level_state(), Refs(speed=1.0, yaw=0.0, z=-15.0), DT)
    assert out.pitch_ref > 0.0
    assert out.elevator < 0.0


def test_pitch_reference_is_limited():
    c = TorpedoController(default_params())
    out = c.update(level_state(z=-1.0), Refs(speed=1.0, yaw=0.0, z=-40.0), DT)
    assert out.pitch_ref == pytest.approx(math.radians(25.0))


def test_fins_are_saturated():
    c = TorpedoController(default_params())
    out = c.update(level_state(u=0.3), Refs(speed=1.0, yaw=math.pi / 2, z=-40.0), DT)
    assert abs(out.rudder) <= 0.26 and abs(out.elevator) <= 0.26


def test_stop_means_propeller_off():
    c = TorpedoController(default_params())
    out = c.update(level_state(u=0.8), Refs(speed=0.0, yaw=0.0, z=-10.0), DT)
    assert out.thrust == 0.0


def test_depth_reference_cannot_go_above_surface():
    c = TorpedoController(default_params())
    out = c.update(level_state(z=-2.0), Refs(speed=1.0, yaw=0.0, z=5.0), DT)
    assert out.z_ref == 0.0


def test_altitude_mode_uses_seabed():
    c = TorpedoController(default_params())
    out = c.update(level_state(z=-20.0, altitude=15.0), Refs(speed=1.0, yaw=0.0, z=-20.0, altitude=10.0), DT)
    assert out.z_ref == pytest.approx(-25.0)
    assert out.pitch_ref < 0.0 or out.pitch_ref > 0.0     # it is acting
    # altitude reference without a valid altitude falls back to depth mode
    out = c.update(level_state(z=-20.0, altitude=None), Refs(speed=1.0, yaw=0.0, z=-20.0, altitude=10.0), DT)
    assert out.z_ref == pytest.approx(-20.0)
