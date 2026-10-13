"""Gazebo server with the underwater world. No GUI here, GUI is a separate container."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory('auv_sim')

    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')

    resource_path = os.pathsep.join(
        p for p in [os.path.join(share, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p
    )

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', verbosity,
             PathJoinSubstitution([share, 'worlds', world])],
        additional_env={'GZ_SIM_RESOURCE_PATH': resource_path},
        output='screen',
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='underwater.sdf',
                              description='world file inside auv_sim/worlds'),
        DeclareLaunchArgument('verbosity', default_value='2',
                              description='gz sim console verbosity, 0 to 4'),
        gz_server,
    ])
