"""
Demo launch — runs the full pipeline with a waypoint client that automatically
calls smooth → generate → execute services/actions.

Usage:
  ros2 launch smooth_nav_bringup demo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('smooth_nav_bringup')

    # Main bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'smooth_nav.launch.py')
        ),
    )

    return LaunchDescription([
        bringup_launch,
        # Future: add a TimerAction that launches a waypoint_client_node
        # after a few seconds to auto-run the demo pipeline
    ])
