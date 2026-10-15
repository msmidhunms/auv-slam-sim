import math

from auv_mission.mission import Config, fls_obstacle_range, Inputs, Mission
import pytest

WAYPOINTS = [(0.0, 0.0), (100.0, 0.0), (100.0, 25.0), (0.0, 25.0)]


def surface(t, fix=True, **kw):
    return Inputs(t=t, x=-20.0, y=-20.0, z=-0.1, yaw=0.0, gps_fix=fix, **kw)


def started(config=None):
    m = Mission(config or Config(), WAYPOINTS)
    m.start(0.0)
    return m


def dived(m, t=20.0):
    """Get through PRE_DIVE and DIVE into SURVEY."""
    m.step(surface(0.0))
    m.step(surface(t))
    assert m.state == 'DIVE'
    m.step(Inputs(t=t + 30, x=-10.0, y=-10.0, z=-14.5, yaw=0.0))
    assert m.state == 'SURVEY'
    return t + 30


def test_idle_commands_nothing():
    m = Mission(Config(), WAYPOINTS)
    assert m.step(surface(0.0)) is None


def test_pre_dive_waits_for_gps():
    m = started()
    cmd = m.step(surface(0.0, fix=False))
    assert m.state == 'PRE_DIVE' and cmd.speed == 0.0
    m.step(surface(5.0))                     # fix starts here
    m.step(surface(14.0))
    assert m.state == 'PRE_DIVE'
    m.step(surface(15.0))                    # 10 s of fix
    assert m.state == 'DIVE'


def test_dive_goes_to_transit_depth():
    m = started()
    m.step(surface(0.0))
    m.step(surface(10.0))
    cmd = m.step(Inputs(t=11.0, x=-20.0, y=-20.0, z=-3.0, yaw=0.0))
    assert m.state == 'DIVE'
    assert cmd.depth == -15.0 and cmd.speed == 1.0
    assert cmd.heading == pytest.approx(math.atan2(20.0, 20.0))


def test_dive_timeout_aborts():
    m = started()
    m.step(surface(0.0))
    m.step(surface(10.0))
    m.step(Inputs(t=400.0, x=-20.0, y=-20.0, z=-3.0, yaw=0.0))
    assert m.state == 'ABORT' and 'dive' in m.reason


def test_survey_uses_altitude_with_dvl_lock_and_depth_without():
    m = started()
    t = dived(m)
    cmd = m.step(Inputs(t=t + 1, x=-5.0, y=-5.0, z=-20.0, yaw=0.0, altitude=10.0))
    assert cmd.altitude == 10.0 and cmd.depth is None
    cmd = m.step(Inputs(t=t + 2, x=-4.0, y=-5.0, z=-20.0, yaw=0.0, altitude=None))
    assert cmd.depth == -15.0 and cmd.altitude is None


def test_survey_walks_the_waypoints_then_returns():
    m = started()
    t = dived(m)
    for i, (x, y) in enumerate(WAYPOINTS):
        m.step(Inputs(t=t + 10 * (i + 1), x=x, y=y, z=-30.0, yaw=0.0, altitude=10.0))
    assert m.state == 'RETURN'
    m.step(Inputs(t=t + 100, x=-18.0, y=-18.0, z=-15.0, yaw=0.0, altitude=10.0))
    assert m.state == 'END'
    cmd = m.step(Inputs(t=t + 101, x=-18.0, y=-18.0, z=-15.0, yaw=0.0))
    assert cmd.speed == 0.0 and cmd.depth == 0.0


def test_surfacing_only_at_lane_end_then_dives_again():
    m = started(Config(surface_interval=100.0))
    t = dived(m)
    # reach waypoint 1 (lane start) late: surfacing is due, but not in the middle of a lane
    m.step(Inputs(t=t + 200, x=0.0, y=0.0, z=-30.0, yaw=0.0, altitude=10.0))
    assert m.state == 'SURVEY'
    m.step(Inputs(t=t + 300, x=100.0, y=0.0, z=-30.0, yaw=0.0, altitude=10.0))   # lane end
    assert m.state == 'SURFACE_FOR_FIX'
    cmd = m.step(Inputs(t=t + 301, x=100.0, y=0.0, z=-20.0, yaw=0.0))
    assert cmd.depth == 0.0 and cmd.speed > 0.0             # climbing with some speed
    m.step(Inputs(t=t + 400, x=100.0, y=0.0, z=-0.1, yaw=0.0, gps_fix=True))
    m.step(Inputs(t=t + 416, x=100.0, y=0.0, z=-0.1, yaw=0.0, gps_fix=True))
    assert m.state == 'DIVE'
    assert m.index == 2                                      # resumes with the next lane


@pytest.mark.parametrize('inputs, reason', [
    (dict(leak=True), 'leak'),
    (dict(battery=0.1), 'battery'),
    (dict(z=-46.0), 'deep'),
    (dict(x=200.0), 'area'),
    (dict(obstacle_range=5.0), 'obstacle'),
])
def test_aborts(inputs, reason):
    m = started()
    t = dived(m)
    values = dict(t=t + 1, x=10.0, y=0.0, z=-20.0, yaw=0.0, altitude=10.0)
    values.update(inputs)
    m.step(Inputs(**values))
    assert m.state == 'ABORT' and reason in m.reason


def test_low_altitude_has_to_persist():
    m = started()
    t = dived(m)
    m.step(Inputs(t=t + 1, x=10.0, y=0.0, z=-30.0, yaw=0.0, altitude=2.0))
    assert m.state == 'SURVEY'
    m.step(Inputs(t=t + 3.5, x=12.0, y=0.0, z=-30.0, yaw=0.0, altitude=2.0))
    assert m.state == 'ABORT' and 'seabed' in m.reason


def test_dvl_loss_aborts_only_after_a_while():
    m = started()
    t = dived(m)
    m.step(Inputs(t=t + 1, x=10.0, y=0.0, z=-30.0, yaw=0.0, altitude=None))
    m.step(Inputs(t=t + 50, x=20.0, y=0.0, z=-30.0, yaw=0.0, altitude=None))
    assert m.state == 'SURVEY'
    m.step(Inputs(t=t + 62, x=30.0, y=0.0, z=-30.0, yaw=0.0, altitude=None))
    assert m.state == 'ABORT' and 'DVL' in m.reason


def test_abort_climbs_then_stops_at_surface():
    m = started()
    t = dived(m)
    m.abort('manual', t)
    cmd = m.step(Inputs(t=t + 1, x=10.0, y=0.0, z=-20.0, yaw=0.3))
    assert cmd.speed > 0.0 and cmd.depth == 0.0 and cmd.heading == 0.3
    cmd = m.step(Inputs(t=t + 60, x=10.0, y=0.0, z=-0.2, yaw=0.3))
    assert cmd.speed == 0.0
    assert m.status == 'ABORT: manual'


def test_fls_ignores_seabed_and_sees_obstacle():
    altitude = 10.0
    seabed = [(15.0, 0.0, -10.0), (20.0, 3.0, -10.2)]
    assert fls_obstacle_range(seabed, altitude, math.radians(20), 1.5) == math.inf
    wreck = seabed + [(12.0, 1.0, -6.0)]
    assert fls_obstacle_range(wreck, altitude, math.radians(20), 1.5) == pytest.approx(math.hypot(12, 1))
    to_the_side = seabed + [(5.0, 8.0, -5.0)]
    assert fls_obstacle_range(to_the_side, altitude, math.radians(20), 1.5) == math.inf
    assert fls_obstacle_range(wreck, None, math.radians(20), 1.5) == math.inf
