"""Depth, heading, attitude and velocity controller.

Works like the depth hold mode of a ROV autopilot:
  - roll and pitch are held level
  - heading and depth are held, and the commands are moving their references
  - surge and sway velocity are tracked using the DVL

Feedback is taken directly from the sensors (AHRS, depth sensor, DVL) and not
from localization, so that the vehicle stays stable even if localization or
SLAM is not running.

cmd_vel:  linear.x  surge velocity, m/s
          linear.y  sway velocity, m/s
          linear.z  depth reference rate, m/s (positive is up)
          angular.z heading reference rate, rad/s
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from auv_control.pid import clamp, Pid, wrap_angle


def rpy_from_quat(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        p = self.declare_parameter

        def pid(name, kp, ki, kd, i_limit, out_limit, i_zone=1.0e9):
            return Pid(p(f'{name}.kp', kp).value, p(f'{name}.ki', ki).value,
                       p(f'{name}.kd', kd).value, p(f'{name}.integral_limit', i_limit).value,
                       p(f'{name}.output_limit', out_limit).value,
                       p(f'{name}.integral_zone', i_zone).value)

        self.surge = pid('surge', 30.0, 10.0, 0.0, 20.0, 80.0)
        self.sway = pid('sway', 40.0, 10.0, 0.0, 20.0, 80.0)
        self.depth = pid('depth', 40.0, 5.0, 45.0, 15.0, 80.0, 0.3)
        self.roll = pid('roll', 8.0, 0.0, 2.0, 0.0, 10.0)
        self.pitch = pid('pitch', 15.0, 0.0, 4.0, 0.0, 12.0)
        self.yaw = pid('yaw', 15.0, 1.0, 4.0, 2.0, 12.0)

        # drag feed forward for the velocity loops: F = linear * v + quadratic * v * |v|
        self.surge_drag = (p('surge.drag_linear', 4.03).value, p('surge.drag_quadratic', 18.18).value)
        self.sway_drag = (p('sway.drag_linear', 6.22).value, p('sway.drag_quadratic', 21.66).value)

        self.max_speed = p('max_speed', 1.0).value                  # m/s, surge and sway
        self.max_heave_rate = p('max_heave_rate', 0.5).value        # m/s
        self.max_yaw_rate = p('max_yaw_rate', 0.8).value            # rad/s
        self.min_depth = p('min_depth', 0.3).value                  # m below the surface
        self.max_depth = p('max_depth', 9.0).value                  # m below the surface
        self.cmd_timeout = p('cmd_timeout', 1.0).value              # s
        self.dvl_timeout = p('dvl_timeout', 0.5).value              # s
        self.enabled = p('enabled', True).value

        self.cmd = Twist()
        self.cmd_stamp = None
        self.z = None
        self.velocity = None            # body frame (u, v, w) from DVL
        self.velocity_stamp = None
        self.z_ref = None
        self.yaw_ref = None
        self.last_stamp = None

        self.wrench_pub = self.create_publisher(WrenchStamped, 'wrench_cmd', 10)
        self.depth_ref_pub = self.create_publisher(Float64, 'controller/depth_reference', 10)
        self.heading_ref_pub = self.create_publisher(Float64, 'controller/heading_reference', 10)

        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'depth/pose', self.on_depth, 10)
        self.create_subscription(TwistWithCovarianceStamped, 'dvl/twist', self.on_dvl, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        # absolute setpoints, used by the mission node
        self.create_subscription(Float64, 'depth_setpoint', self.on_depth_setpoint, 10)
        self.create_subscription(Float64, 'heading_setpoint', self.on_heading_setpoint, 10)
        self.create_service(SetBool, 'controller/enable', self.on_enable)

    # --- inputs ---------------------------------------------------------------

    def on_depth(self, msg):
        self.z = msg.pose.pose.position.z

    def on_dvl(self, msg):
        v = msg.twist.twist.linear
        self.velocity = (v.x, v.y, v.z)
        self.velocity_stamp = Time.from_msg(msg.header.stamp)

    def on_cmd_vel(self, msg):
        self.cmd = msg
        self.cmd_stamp = self.last_stamp

    def on_depth_setpoint(self, msg):
        # setpoint is given as z in world frame, negative below the surface
        self.z_ref = clamp(msg.data, -self.max_depth, -self.min_depth)

    def on_heading_setpoint(self, msg):
        self.yaw_ref = wrap_angle(msg.data)

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.reset()
            self.publish_wrench(self.last_stamp, [0.0] * 6)
        response.success = True
        response.message = 'controller enabled' if self.enabled else 'controller disabled'
        return response

    def reset(self):
        for pid in (self.surge, self.sway, self.depth, self.roll, self.pitch, self.yaw):
            pid.reset()
        self.z_ref = None
        self.yaw_ref = None

    # --- control loop, runs on every IMU sample (50 Hz in sim time) -----------

    def on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        dt = 0.0
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp
        if dt < 0.0 or dt > 0.5:        # simulation reset or a long gap
            self.reset()
            return
        if not self.enabled or self.z is None or dt == 0.0:
            return

        roll, pitch, yaw = rpy_from_quat(msg.orientation)
        rates = msg.angular_velocity

        # hold the place where the controller was started
        if self.z_ref is None:
            self.z_ref = clamp(self.z, -self.max_depth, -self.min_depth)
        if self.yaw_ref is None:
            self.yaw_ref = yaw

        cmd = self.active_command(stamp)
        u_cmd = clamp(cmd.linear.x, -self.max_speed, self.max_speed)
        v_cmd = clamp(cmd.linear.y, -self.max_speed, self.max_speed)
        w_cmd = clamp(cmd.linear.z, -self.max_heave_rate, self.max_heave_rate)
        r_cmd = clamp(cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        self.z_ref = clamp(self.z_ref + w_cmd * dt, -self.max_depth, -self.min_depth)
        self.yaw_ref = wrap_angle(self.yaw_ref + r_cmd * dt)

        u, v, w, dvl_ok = self.body_velocity(stamp)
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)

        # attitude
        torque_x = self.roll.update(-roll, -rates.x, dt)
        torque_y = self.pitch.update(-pitch, -rates.y, dt)
        torque_z = self.yaw.update(wrap_angle(self.yaw_ref - yaw), r_cmd - rates.z, dt)

        # depth, in world frame. Vertical velocity is the DVL velocity rotated to world z
        w_world = -sp * u + cp * sr * v + cp * cr * w
        force_z_world = self.depth.update(self.z_ref - self.z, w_cmd - w_world, dt)

        # horizontal velocity, in body frame. Without DVL it is feed forward alone
        force_x = self.drag(u_cmd, self.surge_drag)
        force_y = self.drag(v_cmd, self.sway_drag)
        if dvl_ok:
            force_x += self.surge.update(u_cmd - u, 0.0, dt)
            force_y += self.sway.update(v_cmd - v, 0.0, dt)

        # world z force expressed in body frame
        force_x += -sp * force_z_world
        force_y += cp * sr * force_z_world
        force_z = cp * cr * force_z_world

        self.publish_wrench(stamp, [force_x, force_y, force_z, torque_x, torque_y, torque_z])
        self.depth_ref_pub.publish(Float64(data=self.z_ref))
        self.heading_ref_pub.publish(Float64(data=self.yaw_ref))

    def active_command(self, stamp):
        """Latest cmd_vel, or zero if it is old. Vehicle then holds position."""
        if self.cmd_stamp is None:
            return Twist()
        if (stamp - self.cmd_stamp).nanoseconds * 1e-9 > self.cmd_timeout:
            return Twist()
        return self.cmd

    def body_velocity(self, stamp):
        if self.velocity_stamp is None:
            return 0.0, 0.0, 0.0, False
        if (stamp - self.velocity_stamp).nanoseconds * 1e-9 > self.dvl_timeout:
            return 0.0, 0.0, 0.0, False
        return (*self.velocity, True)

    @staticmethod
    def drag(velocity, coefficients):
        linear, quadratic = coefficients
        return linear * velocity + quadratic * velocity * abs(velocity)

    def publish_wrench(self, stamp, wrench):
        msg = WrenchStamped()
        if stamp is not None:
            msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_link'
        (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
         msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z) = wrench
        self.wrench_pub.publish(msg)


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
