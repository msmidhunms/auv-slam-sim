"""EKF dead reckoning, publishes /odometry/filtered and the odom -> base_link TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_localization'), 'config', 'ekf.yaml')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # Ground truth from the simulator is in `world` frame. The filter starts at
    # x = y = 0 and the vehicle is spawned at x = y = 0 of the world, heading and
    # depth are absolute, so both frames are the same as long as the simulation
    # and the filter are started together. Needed only to show the ground truth
    # in RViz along with the estimate.
    # With SLAM running the child frame is given as `map`, without SLAM it is `odom`.
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_tf',
        arguments=['--frame-id', 'world', '--child-frame-id', LaunchConfiguration('world_child_frame')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('publish_world_tf')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('publish_world_tf', default_value='true',
                              description='publish the static world -> odom TF for visualisation'),
        DeclareLaunchArgument('world_child_frame', default_value='odom'),
        ekf,
        world_tf,
    ])
