"""robot_state_publisher with the vehicle URDF, gives /robot_description and the static TF."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('auv_description'), 'urdf', 'auv.urdf.xacro'])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([state_publisher])
