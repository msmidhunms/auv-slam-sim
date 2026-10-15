"""Survey pattern and path following maths, free from ROS so that it can be unit tested."""

import math


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def swath_width(altitude, half_angle):
    """Seabed width covered by the multibeam at this altitude (flat seabed)."""
    return 2.0 * altitude * math.tan(half_angle)


def lane_spacing(altitude, half_angle, overlap):
    """Distance between survey lanes so that neighbouring swaths overlap by `overlap` (0..1)."""
    return swath_width(altitude, half_angle) * (1.0 - overlap)


def lane_order(count, spacing, turn_radius):
    """Order in which the lanes are run.

    A torpedo AUV cannot turn tighter than its turning circle. If the lanes are
    closer than the turning diameter, going to the neighbouring lane needs a
    loop, so every second lane is skipped and the skipped ones are run on the
    way back (interleaved pattern), like survey planners do.
    """
    if spacing >= 2.0 * turn_radius or count <= 2:
        return list(range(count))
    forward = list(range(0, count, 2))
    back = [i for i in range(count - 1, -1, -1) if i % 2 == 1]
    return forward + back


def survey_lanes(center, length, count, spacing, heading, lead_in, turn_radius):
    """Waypoints of a lane survey: start and end of each lane, in the order they are run.

    Lanes are `length` long along `heading` and stacked to the left of it with
    `spacing` between them, centred on `center`. Each lane is extended by
    `lead_in` at both ends, so the vehicle is settled on the line before the
    data part starts. Direction alternates from lane to lane.
    """
    if length <= 0 or count < 1 or spacing <= 0:
        raise ValueError('length, count and spacing have to be positive')
    ux, uy = math.cos(heading), math.sin(heading)          # along the lanes
    vx, vy = -uy, ux                                       # across, to the left
    half = length / 2.0 + lead_in
    offset0 = -(count - 1) * spacing / 2.0

    waypoints = []
    for n, lane in enumerate(lane_order(count, spacing, turn_radius)):
        off = offset0 + lane * spacing
        cx, cy = center[0] + vx * off, center[1] + vy * off
        a = (cx - ux * half, cy - uy * half)
        b = (cx + ux * half, cy + uy * half)
        waypoints += [a, b] if n % 2 == 0 else [b, a]
    return waypoints


def path_length(waypoints):
    return sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))


def line_of_sight(position, start, end, lookahead):
    """Line of sight guidance along the segment start -> end.

    Returns (heading, cross_track, along_track, length). Cross track error is
    positive when the vehicle is to the left of the line, and the heading then
    points back to the right towards a point `lookahead` ahead on the line.
    """
    sx, sy = start
    dx, dy = end[0] - sx, end[1] - sy
    length = math.hypot(dx, dy)
    path_angle = math.atan2(dy, dx)
    if length < 1e-6:
        return math.atan2(end[1] - position[1], end[0] - position[0]), 0.0, 0.0, 0.0
    px, py = position[0] - sx, position[1] - sy
    along = (px * dx + py * dy) / length
    cross = (dx * py - dy * px) / length
    heading = wrap_angle(path_angle - math.atan2(cross, lookahead))
    return heading, cross, along, length
