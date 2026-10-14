"""Speed, heading and depth / altitude controller of the torpedo AUV.

Feedback is taken directly from the sensors (AHRS, depth sensor, DVL) and not
from localisation, so the vehicle stays under control even if localisation or
SLAM is down. Control laws are in torpedo.py.

Setpoints (absolute, used by the mission):
  speed_setpoint     m/s
  heading_setpoint   rad
  depth_setpoint     z in m, negative below the surface, switches to depth mode
  altitude_setpoint  m above the seabed, switches to altitude mode

cmd_vel (teleop):
  linear.x  speed setpoint, m/s
  linear.z  depth reference rate, m/s (positive is up)
  angular.z heading reference rate, rad/s

If no setpoint or cmd_vel is coming for `setpoint_timeout`, the propeller is
stopped and the vehicle comes up by its own buoyancy, which is the normal safe
behaviour of an AUV.
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from auv_control.pid import clamp, wrap_angle
from auv_control.torpedo import Refs, State, TorpedoController

GAIN_GROUPS = ('speed', 'heading', 'depth', 'pitch')
GAIN_KEYS = ('kp', 'ki', 'kd', 'integral_limit', 'output_limit', 'integral_zone')
SCALAR_KEYS = ('nominal_speed', 'min_speed', 'max_schedule', 'drag_linear', 'drag_quadratic',
               'min_thrust', 'max_thrust', 'max_depth', 'fin_limit')


def rpy_from_quat(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        params = {k: self.declare_parameter(k, 0.0).value for k in SCALAR_KEYS}
        params['max_pitch'] = math.radians(self.declare_parameter('max_pitch_deg', 25.0).value)
        for group in GAIN_GROUPS:
            params[group] = {}
            for key in GAIN_KEYS:
                default = 1.0e9 if key == 'integral_zone' else 0.0
                params[group][key] = self.declare_parameter(f'{group}.{key}', default).value
        self.control = TorpedoController(params)

        self.max_speed = self.declare_parameter('max_speed', 1.6).value
        self.setpoint_timeout = self.declare_parameter('setpoint_timeout', 5.0).value
        self.dvl_timeout = self.declare_parameter('dvl_timeout', 1.0).value
        self.enabled = self.declare_parameter('enabled', True).value

        self.speed_ref = 0.0
        self.yaw_ref = None
        self.z_ref = None
        self.altitude_ref = None
        self.cmd_rates = (0.0, 0.0)          # depth rate, heading rate from cmd_vel
        self.last_command = None
        self.z = None
        self.velocity = None
        self.velocity_stamp = None
        self.altitude = None
        self.altitude_stamp = None
        self.last_stamp = None

        self.thrust_pub = self.create_publisher(Float64, 'propeller/cmd_thrust', 10)
        self.rudder_pub = self.create_publisher(Float64, 'fins/rudder/cmd_angle', 10)
        self.elevator_pub = self.create_publisher(Float64, 'fins/elevator/cmd_angle', 10)
        self.ref_pubs = {name: self.create_publisher(Float64, f'controller/{name}', 10)
                         for name in ('speed_reference', 'heading_reference',
                                      'depth_reference', 'pitch_reference')}

        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'depth/pose', self.on_depth, 10)
        self.create_subscription(TwistWithCovarianceStamped, 'dvl/twist', self.on_dvl, 10)
        self.create_subscription(Range, 'dvl/altitude', self.on_altitude, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Float64, 'speed_setpoint', self.on_speed, 10)
        self.create_subscription(Float64, 'heading_setpoint', self.on_heading, 10)
        self.create_subscription(Float64, 'depth_setpoint', self.on_depth_setpoint, 10)
        self.create_subscription(Float64, 'altitude_setpoint', self.on_altitude_setpoint, 10)
        self.create_service(SetBool, 'controller/enable', self.on_enable)

        # if the sensors stop, the control loop stops too. Watchdog on wall time
        # switches the propeller off in that case.
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_tick_wall = None
        self.create_timer(0.5, self.on_watchdog, clock=self.steady)

    # --- inputs ---------------------------------------------------------------

    def on_depth(self, msg):
        self.z = msg.pose.pose.position.z

    def on_dvl(self, msg):
        v = msg.twist.twist.linear
        self.velocity = (v.x, v.y, v.z)
        self.velocity_stamp = Time.from_msg(msg.header.stamp)

    def on_altitude(self, msg):
        if msg.min_range <= msg.range <= msg.max_range:
            self.altitude = msg.range
            self.altitude_stamp = Time.from_msg(msg.header.stamp)

    def touch(self):
        self.last_command = self.last_stamp

    def on_cmd_vel(self, msg):
        self.speed_ref = clamp(msg.linear.x, 0.0, self.max_speed)
        self.cmd_rates = (msg.linear.z, msg.angular.z)
        self.touch()

    def on_speed(self, msg):
        self.speed_ref = clamp(msg.data, 0.0, self.max_speed)
        self.cmd_rates = (0.0, 0.0)
        self.touch()

    def on_heading(self, msg):
        self.yaw_ref = wrap_angle(msg.data)
        self.touch()

    def on_depth_setpoint(self, msg):
        self.z_ref = min(msg.data, 0.0)
        self.altitude_ref = None
        self.touch()

    def on_altitude_setpoint(self, msg):
        self.altitude_ref = max(msg.data, 1.0)
        self.touch()

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.control.reset()
            self.publish_actuators(0.0, 0.0, 0.0)
        response.success = True
        response.message = 'controller enabled' if self.enabled else 'controller disabled'
        return response

    # --- control loop, runs on every IMU sample (50 Hz in sim time) -----------

    def on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        dt = 0.0 if self.last_stamp is None else (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp
        if dt < 0.0 or dt > 0.5:            # simulation reset or a long gap
            self.control.reset()
            return
        if not self.enabled or self.z is None or dt == 0.0:
            return
        self.last_tick_wall = self.steady.now()

        roll, pitch, yaw = rpy_from_quat(msg.orientation)
        rates = msg.angular_velocity
        if self.yaw_ref is None:
            self.yaw_ref = yaw
        if self.z_ref is None:
            self.z_ref = self.z

        # no command for a while -> propeller off, vehicle floats up
        if self.last_command is None or \
                (stamp - self.last_command).nanoseconds * 1e-9 > self.setpoint_timeout:
            self.speed_ref = 0.0
            self.cmd_rates = (0.0, 0.0)

        depth_rate, heading_rate = self.cmd_rates
        self.z_ref = min(self.z_ref + depth_rate * dt, 0.0)
        self.yaw_ref = wrap_angle(self.yaw_ref + heading_rate * dt)

        u, v, w, dvl_ok = self.body_velocity(stamp)
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        state = State(
            u=u, z=self.z, w_world=-sp * u + cp * sr * v + cp * cr * w,
            pitch=pitch, yaw=yaw, q=rates.y, r=rates.z, altitude=self.current_altitude(stamp))
        refs = Refs(speed=self.speed_ref, yaw=self.yaw_ref, z=self.z_ref,
                    altitude=self.altitude_ref)

        out = self.control.update(state, refs, dt, speed_valid=dvl_ok)
        self.publish_actuators(out.thrust, out.rudder, out.elevator)
        for name, value in (('speed_reference', self.speed_ref), ('heading_reference', self.yaw_ref),
                            ('depth_reference', out.z_ref), ('pitch_reference', out.pitch_ref)):
            self.ref_pubs[name].publish(Float64(data=float(value)))

    def body_velocity(self, stamp):
        if self.velocity_stamp is None or \
                (stamp - self.velocity_stamp).nanoseconds * 1e-9 > self.dvl_timeout:
            return 0.0, 0.0, 0.0, False
        return (*self.velocity, True)

    def current_altitude(self, stamp):
        if self.altitude_stamp is None or \
                (stamp - self.altitude_stamp).nanoseconds * 1e-9 > self.dvl_timeout:
            return None
        return self.altitude

    def on_watchdog(self):
        if self.last_tick_wall is None:
            return
        if (self.steady.now() - self.last_tick_wall).nanoseconds * 1e-9 > 1.0:
            self.get_logger().warn('no IMU data, propeller stopped', throttle_duration_sec=10.0)
            self.publish_actuators(0.0, 0.0, 0.0)

    def publish_actuators(self, thrust, rudder, elevator):
        self.thrust_pub.publish(Float64(data=float(thrust)))
        self.rudder_pub.publish(Float64(data=float(rudder)))
        self.elevator_pub.publish(Float64(data=float(elevator)))


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
