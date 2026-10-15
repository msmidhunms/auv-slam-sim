"""RTAB-Map bathymetric SLAM on the multibeam submaps, publishes slam_map -> map and the 3D map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_slam'), 'config', 'rtabmap.yaml')
    new_map = LaunchConfiguration('new_map')

    common = {
        'package': 'rtabmap_slam',
        'executable': 'rtabmap',
        'name': 'rtabmap',
        'namespace': 'rtabmap',
        'parameters': [params, {'use_sim_time': True}],
        'remappings': [('scan_cloud', '/multibeam/submap')],
        'output': 'screen',
    }

    return LaunchDescription([
        DeclareLaunchArgument('new_map', default_value='true',
                              description='delete the old database and start a new map'),
        Node(**common, arguments=['--delete_db_on_start'], condition=IfCondition(new_map)),
        Node(**common, condition=UnlessCondition(new_map)),
    ])
