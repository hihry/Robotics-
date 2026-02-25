#!/usr/bin/env python3
"""
waypoint_publisher.py — Publishes waypoints to the trajectory_node parameters.

Reads a scenario from waypoints.yaml and sets the waypoints_x / waypoints_y
parameters of /trajectory_node via the ROS2 parameter API.

Usage:
  python3 scripts/waypoint_publisher.py              # uses default
  python3 scripts/waypoint_publisher.py --scenario figure_eight
"""

import argparse
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class WaypointPublisher(Node):
    def __init__(self, waypoints_x, waypoints_y):
        super().__init__('waypoint_publisher')

        self.client = self.create_client(
            SetParameters, '/trajectory_node/set_parameters')

        self.get_logger().info('Waiting for /trajectory_node parameter service...')
        self.client.wait_for_service(timeout_sec=10.0)

        # Build parameter request
        request = SetParameters.Request()

        px = Parameter()
        px.name = 'waypoints_x'
        px.value = ParameterValue()
        px.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        px.value.double_array_value = waypoints_x

        py = Parameter()
        py.name = 'waypoints_y'
        py.value = ParameterValue()
        py.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        py.value.double_array_value = waypoints_y

        request.parameters = [px, py]

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            results = future.result().results
            for r in results:
                if not r.successful:
                    self.get_logger().error(f'Parameter set failed: {r.reason}')
                    return
            self.get_logger().info(
                f'Waypoints set: {len(waypoints_x)} points')
        else:
            self.get_logger().error('Service call failed')


def main():
    parser = argparse.ArgumentParser(description='Publish waypoints to trajectory_node')
    parser.add_argument('--scenario', type=str, default='figure_eight',
                       help='Scenario name from waypoints.yaml')
    parser.add_argument('--file', type=str, default='',
                       help='Path to waypoints.yaml')
    args = parser.parse_args()

    # Default waypoints
    scenarios = {
        'simple_s_curve': {
            'x': [0.0, 1.0, 2.0, 3.0, 4.0],
            'y': [0.0, 0.0, 1.0, 1.0, 0.0],
        },
        'figure_eight': {
            'x': [0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 3.0, 2.0, 1.0, 0.0],
            'y': [0.0, 0.0, 0.5, 0.5, 0.0, -1.0, -1.5, -1.5, -1.0, 0.0],
        },
        'square_loop': {
            'x': [0.0, 2.0, 2.0, 0.0, 0.0],
            'y': [0.0, 0.0, 2.0, 2.0, 0.0],
        },
    }

    # Try to load from YAML file
    if args.file:
        with open(args.file, 'r') as f:
            data = yaml.safe_load(f)
        if args.scenario in data:
            wx = data[args.scenario]['waypoints_x']
            wy = data[args.scenario]['waypoints_y']
        else:
            print(f'Scenario "{args.scenario}" not found in {args.file}')
            return
    elif args.scenario in scenarios:
        wx = scenarios[args.scenario]['x']
        wy = scenarios[args.scenario]['y']
    else:
        print(f'Unknown scenario: {args.scenario}')
        print(f'Available: {list(scenarios.keys())}')
        return

    rclpy.init()
    node = WaypointPublisher(wx, wy)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
