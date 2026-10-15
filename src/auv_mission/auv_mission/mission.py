"""Mission state machine of the survey AUV, free from ROS so that it can be unit tested.

States, like a real AUV mission:

  IDLE              nothing is commanded
  PRE_DIVE          at the surface, waiting for a good GPS fix
  DIVE              diving to the transit depth, heading to the first waypoint
  SURVEY            following the lanes, at the survey altitude (depth if no DVL lock)
  SURFACE_FOR_FIX   coming up at the end of a lane for a GPS fix, every surface_interval
  RETURN            going back to the recovery point at the transit depth
  END               propeller off, floating at the surface for recovery
  ABORT             something went wrong: climb to the surface, then propeller off

Abort rules are checked in every active state: leak, low battery, too deep,
too close to the seabed, obstacle ahead, DVL lock lost for too long during the
survey, outside the operating area, mission taking too long.
"""

from dataclasses import dataclass
import math

from auv_mission.survey import line_of_sight

ACTIVE = ('DIVE', 'SURVEY', 'SURFACE_FOR_FIX', 'RETURN')


@dataclass
class Inputs:
    t: float                    # s
    x: float                    # m, map frame
    y: float
    z: float                    # m, negative below the surface
    yaw: float                  # rad
    altitude: float = None      # m above the seabed, None without DVL lock
    gps_fix: bool = False
    battery: float = 1.0        # 0 .. 1
    leak: bool = False
    obstacle_range: float = math.inf    # m, horizontal distance to an obstacle ahead


@dataclass
class Command:
    speed: float                # m/s
    heading: float              # rad
    depth: float = None         # z in m, depth mode
    altitude: float = None      # m, altitude mode


@dataclass
class Config:
    cruise_speed: float = 1.0
    abort_speed: float = 0.6
    transit_depth: float = -15.0
    survey_altitude: float = 10.0
    lookahead: float = 12.0
    switch_distance: float = 6.0
    accept_radius: float = 8.0
    home: tuple = (-20.0, -20.0)
    pre_dive_fix_time: float = 10.0
    surface_fix_time: float = 15.0
    surface_wait_max: float = 90.0
    surface_interval: float = 0.0       # s, 0 means no surfacing during the survey
    surfaced_z: float = -0.5
    dive_timeout: float = 300.0
    mission_timeout: float = 3600.0
    max_depth: float = 45.0
    min_altitude: float = 3.0
    min_altitude_time: float = 2.0
    obstacle_distance: float = 8.0
    dvl_loss_time: float = 60.0
    min_battery: float = 0.15
    area: tuple = (-140.0, -140.0, 140.0, 140.0)     # x_min, y_min, x_max, y_max


class Mission:

    def __init__(self, config, waypoints):
        self.c = config
        self.waypoints = list(waypoints)
        self.state = 'IDLE'
        self.reason = ''
        self.index = 0
        self.segment_start = None
        self.reset_timers()

    def reset_timers(self):
        self.start_time = None
        self.state_time = None
        self.fix_since = None
        self.last_fix_time = None
        self.surfaced_since = None
        self.low_altitude_since = None
        self.dvl_lost_since = None
        self.surface_pending = False

    # --- commands from outside -------------------------------------------------

    def start(self, t):
        self.reset_timers()
        self.index = 0
        self.segment_start = None
        self.reason = ''
        self.start_time = t
        self.enter('PRE_DIVE', t)

    def stop(self):
        self.state = 'IDLE'

    def abort(self, reason, t):
        if self.state in ('ABORT', 'IDLE', 'END'):
            return
        self.reason = reason
        self.enter('ABORT', t)

    def enter(self, state, t):
        self.state = state
        self.state_time = t

    @property
    def status(self):
        if self.state == 'SURVEY':
            return f'SURVEY waypoint {self.index + 1}/{len(self.waypoints)}'
        if self.state == 'ABORT':
            return f'ABORT: {self.reason}'
        return self.state

    # --- one step, called at a fixed rate --------------------------------------

    def step(self, s):
        if self.state == 'IDLE':
            return None
        if s.gps_fix:
            self.last_fix_time = s.t if self.last_fix_time is None or self.state != 'SURVEY' else self.last_fix_time
        if self.state in ACTIVE:
            reason = self.check_aborts(s)
            if reason:
                self.abort(reason, s.t)
        return getattr(self, 'do_' + self.state.lower())(s)

    def check_aborts(self, s):
        c = self.c
        if s.leak:
            return 'leak detected'
        if s.battery < c.min_battery:
            return f'battery low ({s.battery * 100:.0f} %)'
        if s.z < -c.max_depth:
            return f'too deep ({-s.z:.1f} m)'
        if s.t - self.start_time > c.mission_timeout:
            return 'mission timeout'
        if not (c.area[0] <= s.x <= c.area[2] and c.area[1] <= s.y <= c.area[3]):
            return 'outside the operating area'
        if s.obstacle_range < c.obstacle_distance:
            return f'obstacle ahead at {s.obstacle_range:.1f} m'
        if s.altitude is not None and s.altitude < c.min_altitude:
            if self.low_altitude_since is None:
                self.low_altitude_since = s.t
            if s.t - self.low_altitude_since >= c.min_altitude_time:
                return f'too close to the seabed ({s.altitude:.1f} m)'
        else:
            self.low_altitude_since = None
        if self.state == 'SURVEY':
            if s.altitude is None:
                if self.dvl_lost_since is None:
                    self.dvl_lost_since = s.t
                if s.t - self.dvl_lost_since >= c.dvl_loss_time:
                    return 'DVL bottom lock lost'
            else:
                self.dvl_lost_since = None
        return None

    def do_pre_dive(self, s):
        if s.gps_fix:
            if self.fix_since is None:
                self.fix_since = s.t
            if s.t - self.fix_since >= self.c.pre_dive_fix_time:
                self.last_fix_time = s.t
                self.enter('DIVE', s.t)
        else:
            self.fix_since = None
        return Command(speed=0.0, heading=s.yaw, depth=0.0)

    def do_dive(self, s):
        c = self.c
        target = self.waypoints[self.index]
        heading = math.atan2(target[1] - s.y, target[0] - s.x)
        if s.z <= c.transit_depth + 1.0:
            self.segment_start = (s.x, s.y)
            self.dvl_lost_since = None
            self.enter('SURVEY', s.t)
        elif s.t - self.state_time > c.dive_timeout:
            self.abort('dive took too long', s.t)
            return self.do_abort(s)
        return Command(speed=c.cruise_speed, heading=heading, depth=c.transit_depth)

    def do_survey(self, s):
        c = self.c
        if c.surface_interval > 0 and self.last_fix_time is not None and \
                s.t - self.last_fix_time >= c.surface_interval:
            self.surface_pending = True

        target = self.waypoints[self.index]
        heading, _, along, length = line_of_sight((s.x, s.y), self.segment_start, target, c.lookahead)
        if along >= length - c.switch_distance or math.dist((s.x, s.y), target) < c.accept_radius:
            reached = self.index
            self.index += 1
            self.segment_start = target
            if self.index >= len(self.waypoints):
                self.segment_start = (s.x, s.y)
                self.enter('RETURN', s.t)
                return self.do_return(s)
            # surfacing only at the end of a lane, not in the middle of the data
            if self.surface_pending and reached % 2 == 1:
                self.surface_pending = False
                self.surfaced_since = None
                self.fix_since = None
                self.enter('SURFACE_FOR_FIX', s.t)
                return self.do_surface_for_fix(s)
            target = self.waypoints[self.index]
            heading, _, _, _ = line_of_sight((s.x, s.y), self.segment_start, target, c.lookahead)

        if s.altitude is not None:
            return Command(speed=c.cruise_speed, heading=heading, altitude=c.survey_altitude)
        return Command(speed=c.cruise_speed, heading=heading, depth=c.transit_depth)

    def do_surface_for_fix(self, s):
        c = self.c
        if s.z < c.surfaced_z:
            # keep some speed for the elevator while coming up
            return Command(speed=c.abort_speed, heading=s.yaw, depth=0.0)
        if self.surfaced_since is None:
            self.surfaced_since = s.t
        if s.gps_fix:
            if self.fix_since is None:
                self.fix_since = s.t
        else:
            self.fix_since = None
        got_fix = self.fix_since is not None and s.t - self.fix_since >= c.surface_fix_time
        if got_fix or s.t - self.surfaced_since >= c.surface_wait_max:
            self.last_fix_time = s.t
            self.enter('DIVE', s.t)
        return Command(speed=0.0, heading=s.yaw, depth=0.0)

    def do_return(self, s):
        c = self.c
        heading, _, along, length = line_of_sight((s.x, s.y), self.segment_start, c.home, c.lookahead)
        if math.dist((s.x, s.y), c.home) < c.accept_radius or along >= length:
            self.enter('END', s.t)
            return self.do_end(s)
        return Command(speed=c.cruise_speed, heading=heading, depth=c.transit_depth)

    def do_end(self, s):
        return Command(speed=0.0, heading=s.yaw, depth=0.0)

    def do_abort(self, s):
        if s.z < self.c.surfaced_z:
            return Command(speed=self.c.abort_speed, heading=s.yaw, depth=0.0)
        return Command(speed=0.0, heading=s.yaw, depth=0.0)


def fls_obstacle_range(points, altitude, half_angle, clearance):
    """Nearest obstacle ahead from the forward looking sonar.

    points: hit points in base_link (x forward, z up). A point is an obstacle if
    it is inside +-half_angle of the forward direction and stands more than
    `clearance` above the seabed below the vehicle (z > -altitude + clearance),
    i.e. it is not just the seabed. Returns the horizontal distance, inf if none.
    """
    best = math.inf
    if altitude is None:
        return best
    floor = -altitude + clearance
    for x, y, z in points:
        if x <= 0.0 or not math.isfinite(x):
            continue
        if abs(math.atan2(y, x)) <= half_angle and z > floor:
            best = min(best, math.hypot(x, y))
    return best
