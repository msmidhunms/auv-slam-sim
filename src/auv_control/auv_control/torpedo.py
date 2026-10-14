"""Control laws of the torpedo AUV, free from ROS so that they can be unit tested.

The vehicle is under actuated: one propeller, a rudder and an elevator. It is
controlled the way survey AUVs are normally controlled:

  speed     propeller thrust = drag feed forward + PI on the DVL speed
  heading   rudder from a PID on the heading error
  depth     outer loop gives a pitch reference from the depth error,
            inner loop gives the elevator from the pitch error
  altitude  same as depth, the depth reference is made from the DVL altitude
  roll      not controlled, the vehicle is stable in roll by itself (centre of
            buoyancy is above the centre of mass)

Fin force goes with speed square, so the fin loops are gain scheduled with
(nominal speed / speed)^2. Below the minimum speed the fins are not useful and
the schedule is clipped.

Sign conventions (checked in the simulator):
  positive rudder turns the vehicle to the right (negative yaw rate)
  negative elevator pitches the nose down (positive pitch in ROS)
"""

from dataclasses import dataclass
import math

from auv_control.pid import clamp, Pid, wrap_angle


@dataclass
class State:
    u: float            # forward speed through water / over ground, m/s
    z: float            # m, negative below the surface
    w_world: float      # vertical speed, m/s, positive up
    pitch: float        # rad, positive is nose down
    yaw: float          # rad
    q: float            # pitch rate, rad/s
    r: float            # yaw rate, rad/s
    altitude: float = None   # m above the seabed, None if not known


@dataclass
class Refs:
    speed: float        # m/s
    yaw: float          # rad
    z: float            # m, used in depth mode
    altitude: float = None   # m, if given the vehicle follows the seabed


@dataclass
class Output:
    thrust: float       # N
    rudder: float       # rad
    elevator: float     # rad
    pitch_ref: float    # rad, for plotting
    z_ref: float        # m, depth reference actually used (differs in altitude mode)


def gain_schedule(u, nominal_speed, min_speed, max_factor):
    """Fin gain factor, (nominal / u)^2 clipped to max_factor."""
    factor = (nominal_speed / max(abs(u), min_speed)) ** 2
    return min(factor, max_factor)


def drag_thrust(speed, linear, quadratic):
    """Thrust needed to hold this speed against the hull drag."""
    return linear * speed + quadratic * speed * abs(speed)


def steady_speed(thrust, linear, quadratic):
    """Speed at which the hull drag is equal to the thrust (inverse of drag_thrust)."""
    if thrust <= 0.0:
        return 0.0
    return (-linear + math.sqrt(linear * linear + 4.0 * quadratic * thrust)) / (2.0 * quadratic)


class SpeedModel:
    """Forward speed estimated from the propeller command, for dead reckoning when
    the DVL has no bottom lock (near the surface, or too high above the seabed).
    Real AUVs use the same idea with the propeller RPM. First order lag with the
    time constant of the vehicle, it is only good to some 10 cm/s."""

    def __init__(self, linear, quadratic, time_constant):
        self.linear = linear
        self.quadratic = quadratic
        self.time_constant = time_constant
        self.speed = 0.0

    def update(self, thrust, dt):
        target = steady_speed(thrust, self.linear, self.quadratic)
        self.speed += (target - self.speed) * min(1.0, dt / self.time_constant)
        return self.speed


def altitude_to_depth(z, altitude, altitude_ref):
    """Depth reference which puts the vehicle at altitude_ref above the seabed."""
    return z - (altitude - altitude_ref)


class TorpedoController:

    def __init__(self, params):
        """params is a dict, same keys as in config/control.yaml."""
        self.p = params

        def pid(name):
            g = params[name]
            return Pid(g['kp'], g['ki'], g['kd'], g['integral_limit'], g['output_limit'],
                       g.get('integral_zone', float('inf')))

        self.speed_pid = pid('speed')
        self.heading_pid = pid('heading')
        self.depth_pid = pid('depth')
        self.pitch_pid = pid('pitch')

    def reset(self):
        for pid in (self.speed_pid, self.heading_pid, self.depth_pid, self.pitch_pid):
            pid.reset()

    def update(self, state, refs, dt, speed_valid=True):
        p = self.p
        factor = gain_schedule(state.u, p['nominal_speed'], p['min_speed'], p['max_schedule'])

        # --- speed ---
        thrust = drag_thrust(refs.speed, p['drag_linear'], p['drag_quadratic'])
        if speed_valid:
            thrust += self.speed_pid.update(refs.speed - state.u, 0.0, dt)
        if refs.speed <= 0.0:
            thrust = 0.0        # stop means propeller off, not reverse
            self.speed_pid.reset()
        thrust = clamp(thrust, p['min_thrust'], p['max_thrust'])

        # --- heading -> rudder. Positive rudder turns right, so the sign is flipped ---
        yaw_error = wrap_angle(refs.yaw - state.yaw)
        rudder = -factor * self.heading_pid.update(yaw_error, -state.r, dt)

        # --- depth (or altitude) -> pitch reference ---
        z_ref = refs.z
        if refs.altitude is not None and state.altitude is not None:
            z_ref = altitude_to_depth(state.z, state.altitude, refs.altitude)
        z_ref = clamp(z_ref, -p['max_depth'], 0.0)
        # deeper reference -> positive error -> positive pitch -> nose down
        pitch_ref = self.depth_pid.update(state.z - z_ref, state.w_world, dt)
        pitch_ref = clamp(pitch_ref, -p['max_pitch'], p['max_pitch'])

        # --- pitch -> elevator. Negative elevator is nose down, so the sign is flipped ---
        elevator = -factor * self.pitch_pid.update(pitch_ref - state.pitch, -state.q, dt)

        limit = p['fin_limit']
        return Output(thrust=thrust, rudder=clamp(rudder, -limit, limit),
                      elevator=clamp(elevator, -limit, limit), pitch_ref=pitch_ref, z_ref=z_ref)


def default_params():
    """Same values as config/control.yaml, used by the unit tests."""
    return {
        'nominal_speed': 1.0, 'min_speed': 0.4, 'max_schedule': 4.0,
        'drag_linear': 1.0, 'drag_quadratic': 6.2282,
        'min_thrust': -10.0, 'max_thrust': 20.0,
        'max_depth': 45.0, 'max_pitch': math.radians(25.0), 'fin_limit': 0.26,
        'speed': {'kp': 8.0, 'ki': 4.0, 'kd': 0.0, 'integral_limit': 6.0, 'output_limit': 10.0},
        'heading': {'kp': 0.35, 'ki': 0.01, 'kd': 0.8, 'integral_limit': 0.05,
                    'output_limit': 0.26, 'integral_zone': 0.2},
        'depth': {'kp': 0.12, 'ki': 0.02, 'kd': 0.5, 'integral_limit': 0.15,
                  'output_limit': 0.45, 'integral_zone': 0.5},
        'pitch': {'kp': 0.8, 'ki': 0.2, 'kd': 0.8, 'integral_limit': 0.15,
                  'output_limit': 0.26},
    }
