"""Survey mission of the AUV. ROS wrapper around the state machine in mission.py.

Navigation comes from the global EKF (/odometry/global, USBL and GPS aided),
altitude from the DVL, the obstacle range from the forward looking sonar. The
output is setpoints for the controller (speed, heading, depth or altitude).

  ros2 service call /mission/start std_srvs/srv/Trigger
  ros2 service call /mission/stop  std_srvs/srv/Trigger    propeller off, floats up
  ros2 service call /mission/abort std_srvs/srv/Trigger    climb to the surface
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from auv_mission.mission import Command, Config, fls_obstacle_range, Inputs, Mission
from auv_mission.survey import lane_spacing, path_length, survey_lanes

# forward looking sonar mount in base_link, same as in the URDF
FLS_XYZ = (0.70, 0.0, -0.08)
FLS_PITCH = 0.5236


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731

        defaults = Config()
        self.config = Config(
            cruise_speed=p('cruise_speed', defaults.cruise_speed),
            climb_speed=p('climb_speed', defaults.climb_speed),
            climb_heading_tolerance=math.radians(p('climb_heading_tolerance_deg', 20.0)),
            transit_depth=p('transit_depth', defaults.transit_depth),
            survey_altitude=p('survey_altitude', defaults.survey_altitude),
            lookahead=p('lookahead', defaults.lookahead),
            switch_distance=p('switch_distance', defaults.switch_distance),
            accept_radius=p('accept_radius', defaults.accept_radius),
            home=tuple(p('home', list(defaults.home))),
            pre_dive_fix_time=p('pre_dive_fix_time', defaults.pre_dive_fix_time),
            surface_fix_time=p('surface_fix_time', defaults.surface_fix_time),
            surface_wait_max=p('surface_wait_max', defaults.surface_wait_max),
            surface_interval=p('surface_interval', defaults.surface_interval),
            dive_timeout=p('dive_timeout', defaults.dive_timeout),
            mission_timeout=p('mission_timeout', defaults.mission_timeout),
            max_depth=p('max_depth', defaults.max_depth),
            min_altitude=p('min_altitude', defaults.min_altitude),
            obstacle_distance=p('obstacle_distance', defaults.obstacle_distance),
            dvl_loss_time=p('dvl_loss_time', defaults.dvl_loss_time),
            min_battery=p('min_battery', defaults.min_battery),
            area=tuple(p('operating_area', list(defaults.area))),
        )

        spacing = lane_spacing(self.config.survey_altitude,
                               math.radians(p('survey.swath_half_angle_deg', 60.0)),
                               p('survey.overlap', 0.3))
        self.waypoints = survey_lanes(
            (p('survey.center_x', 20.0), p('survey.center_y', 0.0)),
            p('survey.lane_length', 100.0), p('survey.lanes', 4), spacing,
            math.radians(p('survey.heading_deg', 0.0)), p('survey.lead_in', 10.0),
            p('survey.turn_radius', 7.0))
        self.mission = Mission(self.config, self.waypoints)
        self.fls_half_angle = math.radians(p('fls_half_angle_deg', 20.0))
        self.fls_clearance = p('fls_clearance', 2.0)

        self.nav = None
        self.altitude = None
        self.gps_fix = False
        self.battery = 1.0
        self.leak = False
        self.obstacle_range = math.inf
        self.last_status = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {k: self.create_publisher(Float64, f'{k}_setpoint', 10)
                     for k in ('speed', 'heading', 'depth', 'altitude')}
        self.status_pub = self.create_publisher(String, 'mission/status', latched)
        self.path_pub = self.create_publisher(Path, 'mission/path', latched)
        self.obstacle_pub = self.create_publisher(Float64, 'mission/obstacle_range', 10)

        self.create_subscription(Odometry, 'odometry/global', self.on_nav, 10)
        self.create_subscription(Range, 'dvl/altitude', self.on_altitude, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.on_gps, 10)
        self.create_subscription(BatteryState, 'battery/state', self.on_battery, 10)
        self.create_subscription(Bool, 'faults/leak', self.on_leak, 10)
        self.create_subscription(PointCloud2, 'sonar/points', self.on_fls, 5)
        self.create_service(Trigger, 'mission/start', self.on_start)
        self.create_service(Trigger, 'mission/stop', self.on_stop)
        self.create_service(Trigger, 'mission/abort', self.on_abort)
        self.create_timer(0.1, self.on_timer)

        self.publish_path()
        self.publish_status()
        self.get_logger().info(
            f'survey: {len(self.waypoints) // 2} lanes, {spacing:.1f} m apart, '
            f'{path_length(self.waypoints):.0f} m of track at {self.config.survey_altitude:.0f} m altitude')

    # --- inputs ----------------------------------------------------------------

    def on_nav(self, msg):
        self.nav = msg

    def on_altitude(self, msg):
        self.altitude = msg.range if math.isfinite(msg.range) else None

    def on_gps(self, msg):
        self.gps_fix = msg.status.status >= NavSatStatus.STATUS_FIX

    def on_battery(self, msg):
        self.battery = msg.percentage

    def on_leak(self, msg):
        self.leak = msg.data

    def on_fls(self, msg):
        raw = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z')).reshape(-1, 3)
        raw = raw[np.all(np.isfinite(raw), axis=1)]
        c, s = math.cos(FLS_PITCH), math.sin(FLS_PITCH)
        x = c * raw[:, 0] + s * raw[:, 2] + FLS_XYZ[0]
        z = -s * raw[:, 0] + c * raw[:, 2] + FLS_XYZ[2]
        points = np.stack([x, raw[:, 1], z], axis=1)
        self.obstacle_range = fls_obstacle_range(points, self.altitude, self.fls_half_angle,
                                                 self.fls_clearance)
        self.obstacle_pub.publish(Float64(data=min(self.obstacle_range, 100.0)))

    # --- services --------------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_start(self, request, response):
        if self.nav is None:
            response.success = False
            response.message = 'no navigation yet (/odometry/global)'
            return response
        self.mission.start(self.now())
        self.publish_status()
        response.success = True
        response.message = f'mission started, {len(self.waypoints)} waypoints'
        return response

    def on_stop(self, request, response):
        self.mission.stop()
        for name in ('speed',):
            self.pubs[name].publish(Float64(data=0.0))
        self.publish_status()
        response.success = True
        response.message = 'mission stopped, propeller off'
        return response

    def on_abort(self, request, response):
        self.mission.abort('manual abort', self.now())
        self.publish_status()
        response.success = self.mission.state == 'ABORT'
        response.message = self.mission.status
        return response

    # --- 10 Hz -----------------------------------------------------------------

    def on_timer(self):
        if self.nav is None:
            return
        p = self.nav.pose.pose.position
        q = self.nav.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        inputs = Inputs(t=self.now(), x=p.x, y=p.y, z=p.z, yaw=yaw, altitude=self.altitude,
                        gps_fix=self.gps_fix, battery=self.battery, leak=self.leak,
                        obstacle_range=self.obstacle_range)
        command = self.mission.step(inputs)
        self.publish_status()
        if command is not None:
            self.send(command)

    def send(self, cmd: Command):
        self.pubs['speed'].publish(Float64(data=cmd.speed))
        self.pubs['heading'].publish(Float64(data=cmd.heading))
        if cmd.altitude is not None:
            self.pubs['altitude'].publish(Float64(data=cmd.altitude))
        else:
            self.pubs['depth'].publish(Float64(data=cmd.depth))

    def publish_status(self):
        status = self.mission.status
        if status != self.last_status:
            self.status_pub.publish(String(data=status))
            log = self.get_logger().warn if status.startswith('ABORT') else self.get_logger().info
            log(f'mission: {status}')
            self.last_status = status

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.position.z = self.config.transit_depth
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
