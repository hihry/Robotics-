"""
Launch file that starts Gazebo simulation with TurtleBot3 in an empty world.

Usage:
  ros2 launch trajectory_control simulation.launch.py
  ros2 launch trajectory_control simulation.launch.py world:=obstacle_world.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to custom world file (empty = TurtleBot3 default empty world)',
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # TurtleBot3 model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Include the TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                turtlebot3_gazebo_share,
                'launch',
                'empty_world.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),
        world_arg,
        gazebo_launch,
    ])
