"""
Launch file for smooth_nav_ros service nodes (path smoother + trajectory generator).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('smooth_nav_ros'),
        'config',
        'smooth_nav_ros.yaml'
    )

    path_smoother_node = Node(
        package='smooth_nav_ros',
        executable='path_smoother_node',
        name='path_smoother_node',
        parameters=[config],
        output='screen',
    )

    trajectory_generator_node = Node(
        package='smooth_nav_ros',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        path_smoother_node,
        trajectory_generator_node,
    ])
