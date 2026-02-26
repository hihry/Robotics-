"""
Demo launch — runs the full pipeline with a waypoint client that automatically
calls smooth → generate → execute services/actions.

Usage:
  ros2 launch smooth_nav_bringup demo.launch.py
  ros2 launch smooth_nav_bringup demo.launch.py waypoint_set:=figure_eight
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('smooth_nav_bringup')

    waypoint_set_arg = DeclareLaunchArgument(
        'waypoint_set',
        default_value='s_curve',
        description='Waypoint set name from waypoints.yaml (s_curve, straight, square, figure_eight)'
    )

    start_delay_arg = DeclareLaunchArgument(
        'start_delay',
        default_value='8.0',
        description='Seconds to wait for Gazebo + nodes before auto-starting pipeline'
    )

    # Main bringup (Gazebo + RViz + services + controller + safety)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'smooth_nav.launch.py')
        ),
    )

    # Waypoint client — auto-starts the full pipeline after a delay
    waypoint_client = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='smooth_nav_bringup',
                executable='waypoint_client_node.py',
                name='waypoint_client',
                output='screen',
                parameters=[{
                    'waypoint_file': os.path.join(
                        bringup_pkg, 'config', 'waypoints.yaml'),
                    'waypoint_set': LaunchConfiguration('waypoint_set'),
                    'auto_start': True,
                    'start_delay': 3.0,
                    'max_velocity': 0.18,
                    'max_acceleration': 0.5,
                    'smoother_service': '/path_smoother/smooth_path',
                    'generator_service': '/trajectory_generator/generate_trajectory',
                    'tracker_action': '/trajectory_tracker/execute_trajectory',
                }],
            ),
        ],
    )

    return LaunchDescription([
        waypoint_set_arg,
        start_delay_arg,
        bringup_launch,
        waypoint_client,
    ])
