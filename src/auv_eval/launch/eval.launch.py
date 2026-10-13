"""Evaluation node, estimate against ground truth. Simulation only."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    evaluation = Node(
        package='auv_eval',
        executable='eval_node',
        name='eval',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([evaluation])
