"""Survey mission node, started with the /mission/start service."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_mission'), 'config', 'mission.yaml')

    mission = Node(
        package='auv_mission',
        executable='mission_node',
        name='mission',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([mission])
