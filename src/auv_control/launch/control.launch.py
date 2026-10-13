"""Starts one control node. Each node is running in its own container, so the
node is selected with the `node` argument (controller or thruster_allocator)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_control'), 'config', 'control.yaml')
    thrusters = os.path.join(
        get_package_share_directory('auv_description'), 'config', 'thrusters.yaml')
    node = LaunchConfiguration('node')

    control = Node(
        package='auv_control',
        executable=[node, '_node'],
        name=node,
        parameters=[params, {'use_sim_time': True, 'thrusters_file': thrusters}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('node', description='controller or thruster_allocator'),
        control,
    ])
