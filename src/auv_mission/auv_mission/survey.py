"""Survey pattern and guidance maths, free from ROS so that it can be unit tested."""

import math


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def lawnmower(center, length, width, spacing, tie_line=True):
    """Waypoints of a lawnmower pattern.

    Lanes are along x with `length`, and they are stacked along y over `width`
    with `spacing` between them. First lane is the one with the lowest y.

    Neighbouring lanes are run in opposite directions, so the forward looking
    sonar is not seeing the same seabed again on them. With `tie_line` the
    vehicle goes back at the end and runs the first lane once more in the same
    direction, which gives SLAM a proper revisit to close the loop.
    """
    if length <= 0 or width < 0 or spacing <= 0:
        raise ValueError('length and spacing have to be positive')
    cx, cy = center
    lanes = int(math.floor(width / spacing)) + 1
    y0 = cy - (lanes - 1) * spacing / 2.0
    x_low, x_high = cx - length / 2.0, cx + length / 2.0

    waypoints = []
    for lane in range(lanes):
        y = y0 + lane * spacing
        ends = (x_low, x_high) if lane % 2 == 0 else (x_high, x_low)
        waypoints.append((ends[0], y))
        waypoints.append((ends[1], y))

    if tie_line and lanes > 1:
        waypoints.append((x_low, y0))
        waypoints.append((x_high, y0))
    return waypoints


def path_length(waypoints):
    return sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))


def guidance(x, y, yaw, target, cruise_speed, turn_in_place_angle, slow_down_gain, min_speed):
    """Heading and surge speed to reach the target waypoint.

    Vehicle is pointed at the waypoint. If the heading error is large it turns
    on the spot first, otherwise the speed is reduced with the heading error and
    also on approaching the waypoint.

    Returns (heading, surge_speed, distance).
    """
    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    heading = math.atan2(dy, dx)
    error = wrap_angle(heading - yaw)

    if abs(error) > turn_in_place_angle:
        return heading, 0.0, distance

    speed = min(cruise_speed, min_speed + slow_down_gain * distance)
    return heading, speed * math.cos(error), distance
