"""Battery model. Energy goes into the hotel load (computers, sensors, sonars)
and the propeller (thrust x speed / propulsion efficiency). Publishes the state
of charge at 1 Hz.

Fault for testing the mission abort: /faults/battery_low (std_msgs/Bool) drops
the charge to 10 %.
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery')
        self.capacity_wh = self.declare_parameter('capacity_wh', 1000.0).value
        self.hotel_w = self.declare_parameter('hotel_load_w', 25.0).value
        self.efficiency = self.declare_parameter('propulsion_efficiency', 0.4).value
        self.voltage_full = self.declare_parameter('voltage_full', 33.6).value
        self.voltage_empty = self.declare_parameter('voltage_empty', 25.0).value
        self.energy_wh = self.capacity_wh * self.declare_parameter('initial_charge', 1.0).value

        self.thrust = 0.0
        self.speed = 0.0
        self.last_stamp = None
        self.next_publish = None

        self.pub = self.create_publisher(BatteryState, 'battery/state', 10)
        self.power_pub = self.create_publisher(Float64, 'battery/power', 10)
        self.create_subscription(Float64, 'propeller/cmd_thrust', self.on_thrust, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)
        self.create_subscription(Bool, 'faults/battery_low', self.on_fault, 10)

    def on_thrust(self, msg):
        self.thrust = msg.data

    def on_fault(self, msg):
        if msg.data:
            self.energy_wh = min(self.energy_wh, 0.1 * self.capacity_wh)
            self.get_logger().warn('fault injected: battery at 10 %')

    def on_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        self.speed = msg.twist.twist.linear.x
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.next_publish = None          # simulation was reset
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
            if 0.0 < dt < 1.0:
                power = self.hotel_w + abs(self.thrust * self.speed) / self.efficiency
                self.energy_wh = max(0.0, self.energy_wh - power * dt / 3600.0)
                self.power = power
        self.last_stamp = stamp
        if self.next_publish is None or stamp >= self.next_publish:
            self.next_publish = stamp + rclpy.duration.Duration(seconds=1.0)
            self.publish(msg.header.stamp)

    def publish(self, stamp):
        fraction = self.energy_wh / self.capacity_wh
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.percentage = fraction
        msg.voltage = self.voltage_empty + (self.voltage_full - self.voltage_empty) * fraction
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = self.capacity_wh / self.voltage_full     # Ah
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub.publish(msg)
        self.power_pub.publish(Float64(data=getattr(self, 'power', self.hotel_w)))


def main():
    rclpy.init()
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
