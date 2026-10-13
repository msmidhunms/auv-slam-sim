"""Starts one sensor node. Each sensor is running in its own container, so the
node is selected with the `node` argument (ahrs, dvl or depth)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_sensors'), 'config', 'sensors.yaml')
    node = LaunchConfiguration('node')

    sensor = Node(
        package='auv_sensors',
        executable=[node, '_node'],
        name=node,
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('node', description='ahrs, dvl or depth'),
        sensor,
    ])
