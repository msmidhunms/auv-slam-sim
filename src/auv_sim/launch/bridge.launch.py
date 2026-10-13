"""Bridge between Gazebo transport and ROS 2, topics are listed in config/bridge.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('auv_sim'), 'config', 'bridge.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': config}],
        output='screen',
    )

    return LaunchDescription([bridge])
