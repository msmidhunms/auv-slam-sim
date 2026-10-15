"""Navigation filters. Each filter is running in its own container, selected
with the `filter` argument:

  local   AHRS + DVL + propeller speed + depth, odom -> base_link and
          /odometry/filtered. Continuous, but drifts.
  global  same plus USBL and GPS, map -> odom and /odometry/global.
          Absolute, the drift is taken out at every fix.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_localization'), 'config', 'ekf.yaml')

    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf',
        parameters=[params, {'use_sim_time': True}],
        condition=LaunchConfigurationEquals('filter', 'local'),
        output='screen',
    )

    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=[params, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')],
        condition=LaunchConfigurationEquals('filter', 'global'),
        output='screen',
    )

    # Ground truth of the simulator is in the `world` frame. Map frame of the
    # survey is the same as world, the vehicle starts at its origin. This TF is
    # only for showing the ground truth in RViz along with the estimates. It is
    # attached to the top of the tree, which is `slam_map` when SLAM is running.
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
        DeclareLaunchArgument('filter', default_value='local', description='local or global'),
        DeclareLaunchArgument('publish_world_tf', default_value='false',
                              description='publish the static world -> <world_child_frame> TF'),
        DeclareLaunchArgument('world_child_frame', default_value='slam_map'),
        local_ekf,
        global_ekf,
        world_tf,
    ])
