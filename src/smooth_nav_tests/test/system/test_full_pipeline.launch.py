"""
System-level launch test — verifies all nodes come up and services are available.

Uses launch_testing to start the full smooth_nav pipeline and check that
expected topics/services/actions exist within a timeout.
"""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node


def generate_test_description():
    """Launch the smooth_nav_ros nodes (no Gazebo) for a lighter system test."""
    ros_pkg = get_package_share_directory('smooth_nav_ros')

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_pkg, 'launch', 'smooth_nav_ros.launch.py')
            ),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSystemNodesUp(unittest.TestCase):
    """Verify that the service nodes start and advertise their services."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('system_test_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_smoother_service_available(self):
        """Check /path_smoother/smooth_path exists."""
        found = False
        for _ in range(50):  # 5 s
            services = self.node.get_service_names_and_types()
            for name, _ in services:
                if 'smooth_path' in name:
                    found = True
                    break
            if found:
                break
            time.sleep(0.1)
        self.assertTrue(found, "SmoothPath service not found within 5 s")

    def test_generator_service_available(self):
        """Check /trajectory_generator/generate_trajectory exists."""
        found = False
        for _ in range(50):
            services = self.node.get_service_names_and_types()
            for name, _ in services:
                if 'generate_trajectory' in name:
                    found = True
                    break
            if found:
                break
            time.sleep(0.1)
        self.assertTrue(found, "GenerateTrajectory service not found within 5 s")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """All processes should exit cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
