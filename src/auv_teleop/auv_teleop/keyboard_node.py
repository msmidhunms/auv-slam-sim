"""Keyboard teleop.

teleop_twist_keyboard is publishing only once per key press, but the controller
stops the vehicle if cmd_vel is older than one second (safety). So this node
keeps publishing the current command at 10 Hz till it is changed.
"""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

HELP = """
Keyboard teleop
---------------
   w / s : forward / backward        q / e : turn left / right
   a / d : left / right (sway)       r / f : up / down

   Each press moves that axis one step: stop -> moving, or moving -> stop.
   space : stop everything           + / - : speed up / down
   Ctrl-C to quit
"""

# key -> (axis, direction)
KEYS = {
    'w': ('surge', 1), 's': ('surge', -1),
    'a': ('sway', 1), 'd': ('sway', -1),
    'r': ('heave', 1), 'f': ('heave', -1),
    'q': ('yaw', 1), 'e': ('yaw', -1),
}


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.speed = self.declare_parameter('speed', 0.4).value             # m/s
        self.heave_speed = self.declare_parameter('heave_speed', 0.3).value # m/s
        self.yaw_rate = self.declare_parameter('yaw_rate', 0.4).value       # rad/s
        self.scale = 1.0
        self.axes = {'surge': 0, 'sway': 0, 'heave': 0, 'yaw': 0}
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def handle_key(self, key):
        if key in KEYS:
            axis, direction = KEYS[key]
            self.axes[axis] = max(-1, min(1, self.axes[axis] + direction))
        elif key == ' ':
            self.axes = dict.fromkeys(self.axes, 0)
        elif key in ('+', '='):
            self.scale = min(2.5, self.scale * 1.25)
        elif key in ('-', '_'):
            self.scale = max(0.2, self.scale / 1.25)
        else:
            return
        self.print_status()

    def twist(self):
        msg = Twist()
        msg.linear.x = self.axes['surge'] * self.speed * self.scale
        msg.linear.y = self.axes['sway'] * self.speed * self.scale
        msg.linear.z = self.axes['heave'] * self.heave_speed * self.scale
        msg.angular.z = self.axes['yaw'] * self.yaw_rate * self.scale
        return msg

    def print_status(self):
        t = self.twist()
        sys.stdout.write(
            f'\rsurge {t.linear.x:+.2f}  sway {t.linear.y:+.2f}  heave {t.linear.z:+.2f} m/s'
            f'  yaw {t.angular.z:+.2f} rad/s   ')
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
        node.pub.publish(Twist())       # stop the vehicle while leaving
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()
