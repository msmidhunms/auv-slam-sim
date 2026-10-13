"""AHRS model on top of the raw IMU from the simulator.

Gazebo gives a perfect orientation. A real AHRS is getting its heading from the
magnetometer, which is having a slowly varying error under water (vehicle
magnetics, thruster currents). That error is added here, otherwise heading will
be simply the ground truth and localization will look better than it should.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu

from auv_sensors.models import GaussMarkov, quat_from_rpy, quat_multiply


class AhrsNode(Node):

    def __init__(self):
        super().__init__('ahrs')
        self.rp_std = math.radians(self.declare_parameter('roll_pitch_noise_std_deg', 0.1).value)
        self.yaw_std = math.radians(self.declare_parameter('yaw_noise_std_deg', 0.3).value)
        bias_std = math.radians(self.declare_parameter('yaw_bias_std_deg', 1.5).value)
        bias_tau = self.declare_parameter('yaw_bias_time_constant', 300.0).value
        self.gyro_std = self.declare_parameter('gyro_noise_std', 0.002).value
        self.accel_std = self.declare_parameter('accel_noise_std', 0.02).value

        self.rng = random.Random(self.declare_parameter('seed', 11).value)
        self.yaw_bias = GaussMarkov(bias_std, bias_tau, self.rng)
        self.bias_std = bias_std
        self.last_stamp = None

        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.sub = self.create_subscription(Imu, 'imu/raw', self.on_imu, 10)

    def on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        dt = 0.0
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
            if dt < 0:      # simulation was reset
                dt = 0.0
        self.last_stamp = stamp

        yaw_error = self.yaw_bias.step(dt) + self.rng.gauss(0.0, self.yaw_std)
        roll_error = self.rng.gauss(0.0, self.rp_std)
        pitch_error = self.rng.gauss(0.0, self.rp_std)

        q = msg.orientation
        q_true = (q.x, q.y, q.z, q.w)
        # heading error is about the world z axis, roll and pitch error are in body frame
        q_out = quat_multiply(quat_from_rpy(0.0, 0.0, yaw_error), q_true)
        q_out = quat_multiply(q_out, quat_from_rpy(roll_error, pitch_error, 0.0))

        out = Imu()
        out.header = msg.header
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_out
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        out.orientation_covariance[0] = self.rp_std ** 2
        out.orientation_covariance[4] = self.rp_std ** 2
        out.orientation_covariance[8] = self.yaw_std ** 2 + self.bias_std ** 2
        for i in (0, 4, 8):
            out.angular_velocity_covariance[i] = self.gyro_std ** 2
            out.linear_acceleration_covariance[i] = self.accel_std ** 2

        self.pub.publish(out)


def main():
    rclpy.init()
    node = AhrsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
