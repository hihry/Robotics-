"""
Launch the trajectory tracker controller node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('smooth_nav_controller'),
        'config',
        'controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='smooth_nav_controller',
            executable='trajectory_tracker_node',
            name='trajectory_tracker_node',
            parameters=[config],
            output='screen',
        ),
    ])
