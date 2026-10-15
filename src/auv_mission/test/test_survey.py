import math

from auv_mission.survey import (
    lane_order,
    lane_spacing,
    line_of_sight,
    path_length,
    survey_lanes,
    swath_width,
    wrap_angle,
)
import pytest


def test_swath_and_lane_spacing():
    assert swath_width(10.0, math.radians(60)) == pytest.approx(34.64, abs=0.01)
    assert lane_spacing(10.0, math.radians(60), 0.3) == pytest.approx(24.25, abs=0.01)


def test_lane_order_simple_when_turn_fits():
    assert lane_order(4, spacing=24.0, turn_radius=7.0) == [0, 1, 2, 3]


def test_lane_order_interleaved_when_lanes_are_too_close():
    # 10 m spacing but 14 m turning diameter -> skip a lane, come back on the skipped ones
    assert lane_order(5, spacing=10.0, turn_radius=7.0) == [0, 2, 4, 3, 1]
    assert lane_order(4, spacing=10.0, turn_radius=7.0) == [0, 2, 3, 1]


def test_survey_lanes_geometry():
    wp = survey_lanes(center=(0.0, 0.0), length=100.0, count=4, spacing=25.0, heading=0.0,
                      lead_in=10.0, turn_radius=7.0)
    assert len(wp) == 8
    assert wp[0] == pytest.approx((-60.0, -37.5))      # lead in included
    assert wp[1] == pytest.approx((60.0, -37.5))
    assert wp[2] == pytest.approx((60.0, -12.5))       # next lane, opposite direction
    assert wp[3] == pytest.approx((-60.0, -12.5))
    assert path_length(wp) == pytest.approx(4 * 120.0 + 3 * 25.0)


def test_survey_lanes_follow_heading():
    wp = survey_lanes((10.0, 20.0), 50.0, 1, 25.0, math.pi / 2, 0.0, 7.0)
    assert wp[0] == pytest.approx((10.0, -5.0))
    assert wp[1] == pytest.approx((10.0, 45.0))


def test_survey_lanes_rejects_bad_input():
    with pytest.raises(ValueError):
        survey_lanes((0, 0), 0.0, 3, 25.0, 0.0, 10.0, 7.0)


def test_los_on_the_line_points_along_it():
    heading, cross, along, length = line_of_sight((30.0, 0.0), (0.0, 0.0), (100.0, 0.0), 12.0)
    assert heading == pytest.approx(0.0)
    assert cross == pytest.approx(0.0)
    assert along == pytest.approx(30.0)
    assert length == pytest.approx(100.0)


def test_los_left_of_line_steers_right():
    heading, cross, _, _ = line_of_sight((30.0, 5.0), (0.0, 0.0), (100.0, 0.0), 12.0)
    assert cross == pytest.approx(5.0)
    assert heading < 0.0
    assert heading == pytest.approx(-math.atan2(5.0, 12.0))


def test_los_right_of_line_steers_left():
    heading, cross, _, _ = line_of_sight((30.0, -5.0), (0.0, 0.0), (100.0, 0.0), 12.0)
    assert cross < 0.0 and heading > 0.0


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
