"""
Master bringup launch — starts everything needed for the smooth_nav demo.

Launches:
  1. Gazebo simulation with TurtleBot3
  2. Robot state publisher + RViz
  3. Path smoother service node
  4. Trajectory generator service node
  5. Trajectory tracker action server

Usage:
  ros2 launch smooth_nav_bringup smooth_nav.launch.py
  ros2 launch smooth_nav_bringup smooth_nav.launch.py world:=obstacle_world.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    bringup_pkg = get_package_share_directory('smooth_nav_bringup')
    sim_pkg = get_package_share_directory('smooth_nav_simulation')
    desc_pkg = get_package_share_directory('smooth_nav_description')
    ros_pkg = get_package_share_directory('smooth_nav_ros')
    ctrl_pkg = get_package_share_directory('smooth_nav_controller')

    # TurtleBot3 model
    tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='burger'
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='Gazebo world file name'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # 1. Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # 2. Robot description + RViz
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'display.launch.py')
        ),
    )

    # 3. smooth_nav_ros (path smoother + traj generator services)
    ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_pkg, 'launch', 'smooth_nav_ros.launch.py')
        ),
    )

    # 4. Controller (trajectory tracker action server)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'controller.launch.py')
        ),
    )

    return LaunchDescription([
        tb3_model,
        world_arg,
        use_rviz_arg,
        gazebo_launch,
        display_launch,
        ros_launch,
        controller_launch,
    ])
