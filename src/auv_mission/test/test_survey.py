import math

from auv_mission.survey import guidance, lawnmower, path_length, wrap_angle
import pytest

GUIDE = dict(cruise_speed=0.5, turn_in_place_angle=math.radians(25), slow_down_gain=0.4,
             min_speed=0.15)


def test_lawnmower_lanes_and_bounds():
    wp = lawnmower((0.0, 0.0), length=30.0, width=24.0, spacing=6.0, tie_line=False)
    assert len(wp) == 10                                    # 5 lanes, 2 points each
    assert min(x for x, _ in wp) == -15.0 and max(x for x, _ in wp) == 15.0
    assert min(y for _, y in wp) == -12.0 and max(y for _, y in wp) == 12.0


def test_lawnmower_alternates_direction():
    wp = lawnmower((0.0, 0.0), 30.0, 24.0, 6.0, tie_line=False)
    assert wp[0] == (-15.0, -12.0) and wp[1] == (15.0, -12.0)
    assert wp[2] == (15.0, -6.0) and wp[3] == (-15.0, -6.0)


def test_lawnmower_tie_line_repeats_first_lane():
    wp = lawnmower((0.0, 0.0), 30.0, 24.0, 6.0, tie_line=True)
    assert len(wp) == 12
    assert wp[-2:] == wp[:2]


def test_lawnmower_center_offset():
    wp = lawnmower((5.0, 2.0), 10.0, 0.0, 6.0)
    assert wp == [(0.0, 2.0), (10.0, 2.0)]                  # single lane, no tie line


def test_lawnmower_rejects_bad_input():
    with pytest.raises(ValueError):
        lawnmower((0, 0), 30.0, 24.0, 0.0)


def test_path_length():
    assert path_length([(0, 0), (3, 4), (3, 10)]) == pytest.approx(11.0)


def test_guidance_straight_ahead_at_cruise():
    heading, speed, distance = guidance(0, 0, 0.0, (10.0, 0.0), **GUIDE)
    assert heading == pytest.approx(0.0)
    assert speed == pytest.approx(0.5)
    assert distance == pytest.approx(10.0)


def test_guidance_turns_in_place_on_big_error():
    heading, speed, _ = guidance(0, 0, 0.0, (0.0, 10.0), **GUIDE)
    assert heading == pytest.approx(math.pi / 2)
    assert speed == 0.0


def test_guidance_slows_near_waypoint():
    _, speed, _ = guidance(0, 0, 0.0, (0.5, 0.0), **GUIDE)
    assert speed == pytest.approx(0.15 + 0.4 * 0.5)


def test_guidance_heading_wraps():
    # vehicle is pointing at 170 deg, target is at -170 deg: 20 deg error, so it moves
    heading, speed, _ = guidance(0, 0, math.radians(170), (-10.0, -1.763), **GUIDE)
    assert wrap_angle(heading - math.radians(170)) == pytest.approx(math.radians(20), abs=1e-3)
    assert speed > 0.0
