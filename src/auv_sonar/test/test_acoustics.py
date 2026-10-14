import math

from auv_sonar.acoustics import (
    fan_angles,
    grazing_intensity,
    multibeam_swath,
    sidescan_ping,
    waterfall_row,
)
import numpy as np
import pytest


def flat_seabed(angles, altitude):
    """Hit points of a fan over a flat seabed `altitude` below (fan frame: x down, y port)."""
    pts = []
    for a in angles:
        if abs(a) >= math.radians(89):
            pts.append((np.inf, np.inf, np.inf))
            continue
        r = altitude / math.cos(a)
        pts.append((r * math.cos(a), r * math.sin(a), 0.0))
    return np.array(pts)


def test_fan_angles():
    a = fan_angles(5, -1.0, 1.0)
    assert list(a) == pytest.approx([-1.0, -0.5, 0.0, 0.5, 1.0])


def test_multibeam_keeps_inner_beams_only():
    angles = fan_angles(181, math.radians(-85), math.radians(85))
    pts = flat_seabed(angles, 10.0)
    swath = multibeam_swath(pts, angles, math.radians(60), 0.0, 0.0, np.random.default_rng(0))
    assert len(swath) == np.sum(np.abs(angles) <= math.radians(60))
    assert np.allclose(swath[:, 0], 10.0)                 # all at the seabed depth
    assert swath[:, 1].max() == pytest.approx(10.0 * math.tan(math.radians(60)), rel=0.02)


def test_multibeam_noise_grows_outwards():
    angles = np.array([0.0, math.radians(60)] * 2000)
    pts = flat_seabed(angles, 10.0)
    swath = multibeam_swath(pts, angles, math.radians(60), 0.002, 3.0, np.random.default_rng(1))
    inner = swath[angles == 0.0][:, 0]
    outer = swath[angles != 0.0][:, 0]
    assert np.std(outer) > 2 * np.std(inner)


def test_grazing_intensity_is_bright_below_and_dark_far_out():
    angles = fan_angles(101, math.radians(-80), math.radians(80))
    intensity = grazing_intensity(flat_seabed(angles, 10.0))
    nadir = intensity[50]
    far = intensity[-2]
    assert nadir == pytest.approx(1.0, abs=0.01)
    assert far < 0.1


def test_sidescan_has_water_column_and_shadow():
    angles = fan_angles(201, math.radians(-85), math.radians(85))
    pts = flat_seabed(angles, 10.0)
    # a box sticking up on port side hides the seabed behind it
    for i, a in enumerate(angles):
        if math.radians(55) < a < math.radians(70):
            pts[i] *= 0.75
    port, starboard = sidescan_ping(pts, angles, max_range=60.0, bins=240, rng=np.random.default_rng(2))
    bin_size = 60.0 / 240
    assert port[:int(9.5 / bin_size)].max() == 0.0          # nothing before the first echo at 10 m
    assert starboard[:int(9.5 / bin_size)].max() == 0.0
    assert starboard[int(10.5 / bin_size):int(20 / bin_size)].mean() > 0.2
    # box echoes come from 13 to 22 m slant range (0.75 x 10 / cos 55..70 deg), the
    # seabed behind it comes again only from 29 m. In between it is dark: the shadow.
    box = port[int(14 / bin_size):int(21 / bin_size)]
    shadow = port[int(23 / bin_size):int(28 / bin_size)]
    assert box.mean() > 0.1
    assert shadow.max() == 0.0


def test_waterfall_row_layout():
    row = waterfall_row(np.array([1.0, 2.0]), np.array([3.0, 4.0]))
    assert list(row) == [2.0, 1.0, 3.0, 4.0]
