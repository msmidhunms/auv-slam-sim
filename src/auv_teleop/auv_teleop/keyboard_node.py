"""Keyboard teleop for the torpedo AUV.

A torpedo AUV is driven by speed, heading and depth, it cannot move sideways or
hover. This node keeps publishing the current command at 10 Hz, because the
controller stops the propeller if commands stop coming (safety).

cmd_vel: linear.x speed in m/s, linear.z depth rate in m/s (up is positive),
         angular.z heading rate in rad/s
"""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

HELP = """
Keyboard teleop (torpedo AUV)
-----------------------------
   w / s : speed up / down (0 .. 1.6 m/s, steps of 0.2)
   a / d : turn left / right            (while turning left, d goes straight again)
   r / f : go up / down                 (while going up, f holds the depth again)
   space : propeller off, stop turning and hold depth
   Ctrl-C to quit

   The vehicle needs speed to turn and to dive, at 0 m/s it floats up slowly.
"""


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.speed_step = self.declare_parameter('speed_step', 0.2).value      # m/s
        self.max_speed = self.declare_parameter('max_speed', 1.6).value        # m/s
        self.depth_rate = self.declare_parameter('depth_rate', 0.3).value      # m/s
        self.turn_rate = self.declare_parameter('turn_rate', 0.12).value       # rad/s
        self.speed = 0.0
        self.turn = 0           # -1, 0, 1
        self.climb = 0          # -1, 0, 1
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def handle_key(self, key):
        if key == 'w':
            self.speed = min(self.max_speed, round(self.speed + self.speed_step, 2))
        elif key == 's':
            self.speed = max(0.0, round(self.speed - self.speed_step, 2))
        elif key == 'a':
            self.turn = max(-1, min(1, self.turn + 1))
        elif key == 'd':
            self.turn = max(-1, min(1, self.turn - 1))
        elif key == 'r':
            self.climb = max(-1, min(1, self.climb + 1))
        elif key == 'f':
            self.climb = max(-1, min(1, self.climb - 1))
        elif key == ' ':
            self.speed, self.turn, self.climb = 0.0, 0, 0
        else:
            return
        self.print_status()

    def twist(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.linear.z = self.climb * self.depth_rate
        msg.angular.z = self.turn * self.turn_rate
        return msg

    def print_status(self):
        turn = {1: 'left ', 0: 'none ', -1: 'right'}[self.turn]
        climb = {1: 'up  ', 0: 'hold', -1: 'down'}[self.climb]
        sys.stdout.write(f'\rspeed {self.speed:.1f} m/s   turn {turn}   depth {climb}      ')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    if not sys.stdin.isatty():
        node.get_logger().error('keyboard teleop needs a terminal, run it with `make teleop`')
        return

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.print_status()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            # wait for a key for 0.1 s, so the command is going out at 10 Hz
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                key = sys.stdin.read(1)
                if key == '\x03':
                    break
                node.handle_key(key.lower())
            node.pub.publish(node.twist())
    except KeyboardInterrupt:
        pass
    finally:
        # leave with the propeller off, the vehicle comes up by itself
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()
