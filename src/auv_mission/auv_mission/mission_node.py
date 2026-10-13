"""Waypoint mission. Runs a lawnmower survey using the SLAM pose.

It is only the outer loop: it gives heading and depth setpoints and the surge
speed to the controller, and the controller is doing the stabilisation.

  ros2 service call /mission/start std_srvs/srv/Trigger
  ros2 service call /mission/stop std_srvs/srv/Trigger
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from auv_mission.survey import guidance, lawnmower, path_length


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameter
        self.pose_frame = p('pose_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.depth = p('depth', -5.0).value                         # z in m
        self.cruise_speed = p('cruise_speed', 0.5).value            # m/s
        self.accept_radius = p('accept_radius', 1.0).value          # m
        self.turn_in_place_angle = math.radians(p('turn_in_place_angle_deg', 25.0).value)
        self.slow_down_gain = p('slow_down_gain', 0.4).value        # 1/s
        self.min_speed = p('min_speed', 0.15).value                 # m/s
        self.depth_tolerance = p('depth_tolerance', 0.5).value      # m
        self.pose_timeout = p('pose_timeout', 1.0).value            # s

        center = (p('survey.center_x', 0.0).value, p('survey.center_y', 0.0).value)
        self.waypoints = lawnmower(
            center, p('survey.length', 30.0).value, p('survey.width', 24.0).value,
            p('survey.spacing', 6.0).value, p('survey.tie_line', True).value)

        self.state = 'IDLE'
        self.index = 0
        self.start_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.heading_pub = self.create_publisher(Float64, 'heading_setpoint', 10)
        self.depth_pub = self.create_publisher(Float64, 'depth_setpoint', 10)
        self.status_pub = self.create_publisher(String, 'mission/status', latched)
        self.path_pub = self.create_publisher(Path, 'mission/path', latched)

        self.create_service(Trigger, 'mission/start', self.on_start)
        self.create_service(Trigger, 'mission/stop', self.on_stop)
        self.timer = self.create_timer(0.1, self.on_timer)

        self.publish_path()
        self.set_state('IDLE')
        self.get_logger().info(
            f'survey with {len(self.waypoints)} waypoints, {path_length(self.waypoints):.0f} m, '
            f'at z = {self.depth} m')
        if p('autostart', False).value:
            self.set_state('RUNNING')

    # --- services ---------------------------------------------------------------

    def on_start(self, request, response):
        self.index = 0
        self.start_time = None
        self.set_state('RUNNING')
        response.success = True
        response.message = f'mission started, {len(self.waypoints)} waypoints'
        return response

    def on_stop(self, request, response):
        self.set_state('IDLE')
        self.cmd_pub.publish(Twist())
        response.success = True
        response.message = 'mission stopped, vehicle is holding position'
        return response

    def set_state(self, state):
        self.state = state
        text = state
        if state == 'RUNNING':
            text = f'RUNNING waypoint {self.index + 1}/{len(self.waypoints)}'
        self.status_pub.publish(String(data=text))

    # --- guidance loop, 10 Hz ----------------------------------------------------

    def on_timer(self):
        # when not running nothing is published, so teleop can be used
        if self.state != 'RUNNING':
            return

        pose = self.current_pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(
                f'no {self.pose_frame} -> {self.base_frame} pose, waiting',
                throttle_duration_sec=5.0)
            return
        x, y, z, yaw = pose
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        self.depth_pub.publish(Float64(data=self.depth))

        target = self.waypoints[self.index]
        heading, speed, distance = guidance(
            x, y, yaw, target, self.cruise_speed, self.turn_in_place_angle,
            self.slow_down_gain, self.min_speed)

        if distance < self.accept_radius:
            self.index += 1
            if self.index >= len(self.waypoints):
                self.cmd_pub.publish(Twist())
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
                self.get_logger().info(f'mission completed in {elapsed:.0f} s')
                self.set_state('DONE')
                return
            self.get_logger().info(f'waypoint {self.index}/{len(self.waypoints)} reached')
            self.set_state('RUNNING')
            return

        # go to the survey depth first, then start moving
        if abs(z - self.depth) > self.depth_tolerance:
            speed = 0.0

        self.heading_pub.publish(Float64(data=heading))
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)

    def current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.pose_frame, self.base_frame, Time())
        except TransformException:
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.pose_timeout:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, t.z, yaw

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.pose_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.depth
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
