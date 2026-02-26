# Copyright 2026 smooth_nav Authors
# SPDX-License-Identifier: Apache-2.0

"""Launch robot state publisher + RViz for smooth_nav visualization."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('smooth_nav_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger.urdf')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'smooth_nav.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
    ])
