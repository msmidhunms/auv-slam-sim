"""Converts the body wrench command into the thrust command of each thruster."""

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import yaml

from auv_control.allocation import ThrusterAllocator


class ThrusterAllocatorNode(Node):

    def __init__(self):
        super().__init__('thruster_allocator')
        thrusters_file = self.declare_parameter('thrusters_file', '').value
        self.timeout = self.declare_parameter('command_timeout', 0.5).value

        with open(thrusters_file) as f:
            layout = yaml.safe_load(f)
        self.names = [t['name'] for t in layout['thrusters']]
        self.allocator = ThrusterAllocator(layout['thrusters'], layout['max_thrust'])
        self.get_logger().info(
            f'{len(self.names)} thrusters, max thrust {layout["max_thrust"]} N')

        self.pubs = [self.create_publisher(Float64, f'thrusters/{name}/cmd_thrust', 10)
                     for name in self.names]
        # all thrusts in one message, handy for plotting
        self.all_pub = self.create_publisher(Float64MultiArray, 'thrusters/cmd', 10)
        self.sub = self.create_subscription(WrenchStamped, 'wrench_cmd', self.on_wrench, 10)

        self.last_command = None
        self.stopped = True
        # watchdog is on wall time, it has to work even if the simulation clock stops
        self.watchdog = self.create_timer(
            0.1, self.on_watchdog, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.steady_clock = self.watchdog.clock

    def on_wrench(self, msg):
        w = msg.wrench
        thrusts = self.allocator.allocate(
            [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
        self.publish(thrusts)
        self.last_command = self.steady_clock.now()
        self.stopped = False

    def on_watchdog(self):
        if self.stopped or self.last_command is None:
            return
        age = (self.steady_clock.now() - self.last_command).nanoseconds * 1e-9
        if age > self.timeout:
            self.get_logger().warn('no wrench command, stopping the thrusters')
            self.publish([0.0] * len(self.names))
            self.stopped = True

    def publish(self, thrusts):
        for pub, thrust in zip(self.pubs, thrusts):
            pub.publish(Float64(data=float(thrust)))
        self.all_pub.publish(Float64MultiArray(data=[float(t) for t in thrusts]))


def main():
    rclpy.init()
    node = ThrusterAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
