"""Sonar maths, free from ROS so that it can be unit tested.

The simulator traces one acoustic fan across the track: N rays from port to
starboard in the plane perpendicular to the vehicle axis. Fan frame has x
pointing down and y to port, so a ray at angle a from the nadir points along
(cos a, sin a, 0) and a positive angle is to port.

From this one fan two instruments are made, like on a real survey AUV:
  multibeam  the inner part of the fan (+-60 deg), depth points across the track
  sidescan   the whole fan, echo intensity against slant range on each side
"""

import math

import numpy as np


def fan_angles(count, min_angle, max_angle):
    """Angle of each ray in the fan, same order as the simulator gives them."""
    if count == 1:
        return np.array([0.0])
    return np.linspace(min_angle, max_angle, count)


def multibeam_swath(points, angles, max_angle, range_noise, angle_noise_gain, rng):
    """Depth points of one ping.

    points  N x 3 hit points in the fan frame, NaN or inf for no return
    Returns M x 3 points of the beams inside +-max_angle which hit the seabed.
    Range noise grows towards the outer beams (refraction and beam footprint),
    as on real multibeams.
    """
    points = np.asarray(points, dtype=float)
    ranges = np.linalg.norm(points, axis=1)
    valid = np.isfinite(ranges) & (np.abs(angles) <= max_angle)
    if not np.any(valid):
        return np.zeros((0, 3))
    p, r, a = points[valid], ranges[valid], angles[valid]
    sigma = range_noise * r * (1.0 + angle_noise_gain * np.sin(a) ** 2)
    factor = 1.0 + rng.normal(0.0, 1.0, size=r.shape) * sigma / r
    return p * factor[:, None]


def grazing_intensity(points):
    """Echo strength of each hit, from the angle between the ray and the seabed.

    Seabed normal is not known, so it is taken from the neighbouring hits in the
    fan (local slope across the track). Lambert's law: intensity goes with the
    square of the cosine of the incidence angle. A face looking at the sonar
    (front of a rock) is bright, a face turned away is dark.
    Returns an array of 0 .. 1, NaN where there is no return.
    """
    points = np.asarray(points, dtype=float)
    n = len(points)
    out = np.full(n, np.nan)
    finite = np.all(np.isfinite(points), axis=1)
    for i in range(n):
        if not finite[i]:
            continue
        prev_i = i - 1 if i > 0 and finite[i - 1] else i
        next_i = i + 1 if i < n - 1 and finite[i + 1] else i
        if prev_i == next_i:
            out[i] = 0.5
            continue
        tangent = points[next_i] - points[prev_i]
        # seabed normal in the fan plane (x down, y across), turned towards the sonar
        normal = np.array([-tangent[1], tangent[0], 0.0])
        length = np.linalg.norm(normal)
        ray = points[i] / np.linalg.norm(points[i])
        if length < 1e-9:
            out[i] = 0.5
            continue
        normal /= length
        cos_incidence = abs(float(np.dot(normal, -ray)))
        out[i] = cos_incidence ** 2
    return out


def sidescan_ping(points, angles, max_range, bins, rng, speckle=0.35, max_gap=1.5):
    """One sidescan line: intensity against slant range for port and starboard.

    Each ray gives an echo at its slant range. Between two neighbouring rays the
    echo is filled in, unless their ranges jump by more than max_gap, which
    means the second one is hidden behind an object: that stretch stays dark,
    which is the acoustic shadow. Water column (before the first bottom echo)
    is dark too. Speckle is Rayleigh noise, like on a real sidescan.

    Returns (port, starboard), each an array of `bins` values in 0 .. 1, index
    0 is at the vehicle.
    """
    points = np.asarray(points, dtype=float)
    ranges = np.linalg.norm(points, axis=1)
    intensity = grazing_intensity(points)
    bin_size = max_range / bins
    sides = []
    for side in (1.0, -1.0):        # port is positive angle
        line = np.zeros(bins)
        idx = [i for i in np.argsort(np.abs(angles)) if np.sign(angles[i]) in (side, 0.0)]
        for a, b in zip(idx, idx[1:]):
            if not (np.isfinite(ranges[a]) and np.isfinite(ranges[b])):
                continue
            r0, r1 = ranges[a], ranges[b]
            if abs(r1 - r0) > max_gap:
                continue            # shadow or jump, nothing to fill
            lo, hi = sorted((r0, r1))
            b0, b1 = int(lo / bin_size), int(hi / bin_size)
            for k in range(b0, min(b1, bins - 1) + 1):
                t = 0.5 if hi == lo else ((k + 0.5) * bin_size - lo) / (hi - lo)
                t = min(max(t, 0.0), 1.0)
                value = intensity[a] + (intensity[b] - intensity[a]) * t
                line[k] = max(line[k], value)
        line *= rng.rayleigh(speckle, size=bins) / (speckle * math.sqrt(math.pi / 2))
        sides.append(np.clip(line, 0.0, 1.0))
    return sides[0], sides[1]


def waterfall_row(port, starboard):
    """Port is shown on the left with range growing to the left, like on the
    sidescan display of a survey software."""
    return np.concatenate([port[::-1], starboard])
