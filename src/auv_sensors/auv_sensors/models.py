"""Sensor maths, kept free from ROS so that it can be unit tested directly."""

import math

GRAVITY = 9.8


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def velocity_at_offset(linear, angular, offset):
    """Body frame velocity of a point which is at `offset` from the body origin."""
    w_x_r = cross(angular, offset)
    return tuple(v + c for v, c in zip(linear, w_x_r))


def depth_to_pressure(depth, density, atmospheric):
    """Absolute pressure in Pa at the given depth in m (positive down)."""
    return atmospheric + density * GRAVITY * depth


def pressure_to_depth(pressure, density, atmospheric):
    return (pressure - atmospheric) / (density * GRAVITY)


def quat_multiply(a, b):
    """Hamilton product, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class GaussMarkov:
    """First order Gauss-Markov process, used for slowly wandering sensor bias."""

    def __init__(self, std, time_constant, rng):
        self.std = std
        self.tau = time_constant
        self.rng = rng
        self.value = rng.gauss(0.0, std) if std > 0 else 0.0

    def step(self, dt):
        if self.std <= 0 or dt <= 0:
            return self.value
        a = math.exp(-dt / self.tau)
        # driving noise is scaled so that the steady state std stays at self.std
        self.value = a * self.value + self.std * math.sqrt(1 - a * a) * self.rng.gauss(0.0, 1.0)
        return self.value


# --- DVL beams ---------------------------------------------------------------

def dvl_altitude(beam_ranges, beam_angle, min_beams=3):
    """Altitude from the slant ranges of the Janus beams.

    Each beam is tilted by beam_angle from the vertical, so its vertical part is
    range * cos(beam_angle). Beams without bottom return are None. Like a real
    DVL, at least min_beams valid beams are needed for bottom lock.
    Returns the altitude, or None if there is no lock.
    """
    valid = [r for r in beam_ranges if r is not None and math.isfinite(r)]
    if len(valid) < min_beams:
        return None
    return sum(valid) / len(valid) * math.cos(beam_angle)


# --- GPS -----------------------------------------------------------------------

EARTH_RADIUS = 6378137.0


def enu_to_geodetic(x, y, lat0, lon0):
    """Local east / north (m) to latitude / longitude (deg), flat earth around the datum.
    Good to a few cm over the few km of a survey."""
    lat = lat0 + math.degrees(y / EARTH_RADIUS)
    lon = lon0 + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(lat0))))
    return lat, lon


def geodetic_to_enu(lat, lon, lat0, lon0):
    y = math.radians(lat - lat0) * EARTH_RADIUS
    x = math.radians(lon - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
    return x, y


class SurfaceFix:
    """GPS antenna is above water only when the vehicle is at the surface, and
    the receiver needs some time at the surface before it gives a fix."""

    def __init__(self, surface_z, time_to_fix):
        self.surface_z = surface_z
        self.time_to_fix = time_to_fix
        self.surfaced_since = None

    def update(self, z, t):
        """Returns True if a fix is available at time t (s)."""
        if z < self.surface_z:
            self.surfaced_since = None
            return False
        if self.surfaced_since is None:
            self.surfaced_since = t
        return t - self.surfaced_since >= self.time_to_fix


# --- USBL ----------------------------------------------------------------------

SOUND_SPEED = 1500.0     # m/s


def usbl_std(slant_range, relative_error, minimum):
    """Position error (1 sigma, per axis) of a USBL fix, grows with slant range."""
    return max(minimum, relative_error * slant_range)


def usbl_latency(slant_range, processing):
    """Time from the ping till the fix is available on the ship: two way travel
    of sound plus the processing time."""
    return 2.0 * slant_range / SOUND_SPEED + processing
