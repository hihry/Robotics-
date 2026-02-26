# Copyright 2026 smooth_nav Authors
# SPDX-License-Identifier: Apache-2.0

"""
Launch Gazebo with TurtleBot3 for smooth_nav simulation.

Usage:
  ros2 launch smooth_nav_simulation gazebo.launch.py
  ros2 launch smooth_nav_simulation gazebo.launch.py world:=obstacle_world.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('smooth_nav_simulation')

    # TurtleBot3 model environment variable
    turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World file name in smooth_nav_simulation/worlds/'
    )

    world_file = os.path.join(sim_pkg, 'worlds')

    # Gazebo launch from gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': [world_file, '/', LaunchConfiguration('world')],
        }.items(),
    )

    # Spawn TurtleBot3 using turtlebot3_gazebo spawn
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )

    return LaunchDescription([
        turtlebot3_model,
        world_arg,
        gazebo,
        spawn_turtlebot,
    ])
