"""Starts one sonar node. Each node is running in its own container, so the
node is selected with the `node` argument (multibeam, swath_assembler or sidescan)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_sonar'), 'config', 'sonar.yaml')
    node = LaunchConfiguration('node')

    sonar = Node(
        package='auv_sonar',
        executable=[node, '_node'],
        name=node,
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('node', description='multibeam, swath_assembler or sidescan'),
        sonar,
    ])
