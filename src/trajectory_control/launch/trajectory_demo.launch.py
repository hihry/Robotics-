"""
Full demo launch: Gazebo + TurtleBot3 + trajectory_node + RViz2.

Usage:
  ros2 launch trajectory_control trajectory_demo.launch.py
  ros2 launch trajectory_control trajectory_demo.launch.py use_rviz:=false
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
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    pkg_share = FindPackageShare('trajectory_control')
    turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')

    # Controller parameters file
    params_file = PathJoinSubstitution([
        pkg_share, 'config', 'controller_params.yaml'
    ])

    # ---- Gazebo + TurtleBot3 ----
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
        }.items(),
    )

    # ---- Trajectory Node ----
    trajectory_node = Node(
        package='trajectory_control',
        executable='trajectory_node',
        name='trajectory_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # ---- RViz2 ----
    rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'trajectory.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        gazebo_launch,
        trajectory_node,
        rviz_node,
    ])
