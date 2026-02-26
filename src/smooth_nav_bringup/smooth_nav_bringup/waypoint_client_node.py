#!/usr/bin/env python3
"""
waypoint_client_node.py — End-to-end pipeline orchestrator for smooth_nav.

This node is the "brain" that ties the entire navigation pipeline together:

    Waypoints (YAML) ──→ SmoothPath service ──→ GenerateTrajectory service
                                                        │
                                cmd_vel ◄── ExecuteTrajectory action ◄──┘

Usage:
    ros2 run smooth_nav_bringup waypoint_client_node
    ros2 run smooth_nav_bringup waypoint_client_node --ros-args -p waypoint_set:=square

Lifecycle:
    1. Reads waypoints from parameter or /waypoints topic
    2. Calls ~/smooth_path → gets smoothed path
    3. Calls ~/generate_trajectory → gets time-parameterised trajectory
    4. Sends goal to ~/execute_trajectory action → robot moves
    5. Publishes progress on /pipeline_status

Real-world considerations:
    - Retries with exponential backoff on service failures
    - Validates all intermediate results before proceeding
    - Publishes visual markers for debugging in RViz
    - Supports aborting mid-execution via /abort_mission topic
"""

import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Bool, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from smooth_nav_msgs.srv import SmoothPath, GenerateTrajectory
from smooth_nav_msgs.msg import (
    WaypointArray, Waypoint, SmoothedPath, Trajectory, TrajectoryPoint
)
from smooth_nav_msgs.action import ExecuteTrajectory

from ament_index_python.packages import get_package_share_directory


class WaypointClientNode(Node):
    """Orchestrates the full smooth_nav pipeline: smooth → generate → execute."""

    def __init__(self):
        super().__init__('waypoint_client_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('waypoint_set', 's_curve')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('max_velocity', 0.18)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('retry_count', 3)
        self.declare_parameter('retry_delay', 2.0)

        # ── Service clients ─────────────────────────────────────────
        self._cb_group = MutuallyExclusiveCallbackGroup()

        self.smooth_client = self.create_client(
            SmoothPath,
            '/path_smoother_node/smooth_path',
            callback_group=self._cb_group)

        self.traj_client = self.create_client(
            GenerateTrajectory,
            '/trajectory_generator_node/generate_trajectory',
            callback_group=self._cb_group)

        # ── Action client ───────────────────────────────────────────
        self.exec_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/trajectory_tracker_node/execute_trajectory',
            callback_group=self._cb_group)

        # ── Publishers ──────────────────────────────────────────────
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.status_pub = self.create_publisher(String, '/pipeline_status', 10)
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', latching_qos)
        self.traj_marker_pub = self.create_publisher(
            MarkerArray, '/trajectory_markers', latching_qos)

        # ── Abort subscription ──────────────────────────────────────
        self.abort_sub = self.create_subscription(
            Bool, '/abort_mission', self._abort_callback, 10)
        self._abort_requested = False
        self._goal_handle = None

        # ── Auto-start timer ────────────────────────────────────────
        if self.get_parameter('auto_start').value:
            delay = self.get_parameter('start_delay').value
            self.get_logger().info(
                f'Auto-starting pipeline in {delay:.1f} s...')
            self._timer = self.create_timer(delay, self._run_pipeline_once)
        else:
            self.get_logger().info(
                'Waiting for /start_mission (std_msgs/Bool) to begin.')
            self.start_sub = self.create_subscription(
                Bool, '/start_mission', lambda msg: self._run_pipeline_once(), 10)

    # ════════════════════════════════════════════════════════════════
    #  Pipeline orchestration
    # ════════════════════════════════════════════════════════════════

    def _run_pipeline_once(self):
        """Execute the full pipeline once, then destroy the auto-start timer."""
        if hasattr(self, '_timer'):
            self._timer.cancel()
        self._abort_requested = False

        try:
            waypoints = self._load_waypoints()
            self._publish_status('SMOOTHING')
            smoothed = self._call_smooth(waypoints)
            if smoothed is None:
                return

            self._publish_status('GENERATING_TRAJECTORY')
            trajectory = self._call_generate(smoothed)
            if trajectory is None:
                return

            self._publish_status('EXECUTING')
            self._call_execute(trajectory)

        except Exception as e:
            self.get_logger().error(f'Pipeline failed: {e}')
            self._publish_status(f'ERROR: {e}')

    # ── 1. Load waypoints ───────────────────────────────────────────

    def _load_waypoints(self):
        """Load waypoints from parameter file or built-in sets."""
        wp_file = self.get_parameter('waypoints_file').value
        wp_set = self.get_parameter('waypoint_set').value

        if wp_file:
            return self._load_from_yaml(wp_file, wp_set)

        # Try loading from bringup package
        try:
            pkg_dir = get_package_share_directory('smooth_nav_bringup')
            yaml_path = f'{pkg_dir}/config/waypoints.yaml'
            return self._load_from_yaml(yaml_path, wp_set)
        except Exception:
            pass

        # Fallback: built-in waypoints
        self.get_logger().warn(f'Using built-in waypoint set: {wp_set}')
        return self._builtin_waypoints(wp_set)

    def _load_from_yaml(self, path, wp_set):
        """Parse waypoints.yaml and extract the named set."""
        self.get_logger().info(f'Loading waypoints from {path} [set={wp_set}]')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        wp_data = data.get('waypoints', {}).get(wp_set)
        if wp_data is None:
            available = list(data.get('waypoints', {}).keys())
            raise ValueError(
                f'Waypoint set "{wp_set}" not found. Available: {available}')

        xs = wp_data['x']
        ys = wp_data['y']
        if len(xs) != len(ys):
            raise ValueError('x and y arrays must have equal length')

        waypoints = []
        for x, y in zip(xs, ys):
            wp = Waypoint()
            wp.x = float(x)
            wp.y = float(y)
            waypoints.append(wp)

        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from "{wp_set}"')
        self._publish_waypoint_markers(waypoints)
        return waypoints

    def _builtin_waypoints(self, name):
        """Hardcoded fallback waypoints."""
        sets = {
            's_curve': [
                (0.0, 0.0), (1.0, 0.0), (2.0, 0.5), (3.0, 0.5),
                (4.0, 0.0), (4.0, -1.0), (3.0, -1.5), (2.0, -1.5),
                (1.0, -1.0), (0.0, 0.0)
            ],
            'straight': [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
            'square': [
                (0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)
            ],
        }
        pts = sets.get(name, sets['s_curve'])
        waypoints = []
        for x, y in pts:
            wp = Waypoint()
            wp.x = x
            wp.y = y
            waypoints.append(wp)
        self._publish_waypoint_markers(waypoints)
        return waypoints

    # ── 2. Call SmoothPath service ──────────────────────────────────

    def _call_smooth(self, waypoints):
        """Call the path smoother service with retries."""
        retries = self.get_parameter('retry_count').value
        delay = self.get_parameter('retry_delay').value

        for attempt in range(1, retries + 1):
            if self._abort_requested:
                self._publish_status('ABORTED')
                return None

            if not self.smooth_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f'SmoothPath service unavailable (attempt {attempt}/{retries})')
                time.sleep(delay)
                continue

            request = SmoothPath.Request()
            request.waypoints.header.stamp = self.get_clock().now().to_msg()
            request.waypoints.header.frame_id = 'odom'
            request.waypoints.waypoints = waypoints

            self.get_logger().info(
                f'Calling SmoothPath with {len(waypoints)} waypoints...')
            future = self.smooth_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is None:
                self.get_logger().error(
                    f'SmoothPath call failed (attempt {attempt}/{retries})')
                time.sleep(delay)
                continue

            resp = future.result()
            if not resp.success:
                self.get_logger().error(f'SmoothPath rejected: {resp.message}')
                return None

            n_pts = len(resp.smoothed_path.points)
            self.get_logger().info(
                f'Path smoothed: {len(waypoints)} waypoints → {n_pts} points')
            return resp.smoothed_path

        self.get_logger().error('SmoothPath service exhausted all retries')
        self._publish_status('ERROR: SmoothPath unavailable')
        return None

    # ── 3. Call GenerateTrajectory service ──────────────────────────

    def _call_generate(self, smoothed_path):
        """Call the trajectory generator service."""
        retries = self.get_parameter('retry_count').value
        delay = self.get_parameter('retry_delay').value

        for attempt in range(1, retries + 1):
            if self._abort_requested:
                self._publish_status('ABORTED')
                return None

            if not self.traj_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f'GenerateTrajectory service unavailable '
                    f'(attempt {attempt}/{retries})')
                time.sleep(delay)
                continue

            request = GenerateTrajectory.Request()
            request.smoothed_path = smoothed_path
            request.max_velocity = self.get_parameter('max_velocity').value
            request.max_acceleration = self.get_parameter('max_acceleration').value

            self.get_logger().info(
                f'Calling GenerateTrajectory '
                f'(v_max={request.max_velocity}, a_max={request.max_acceleration})...')
            future = self.traj_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is None:
                self.get_logger().error(
                    f'GenerateTrajectory call failed (attempt {attempt}/{retries})')
                time.sleep(delay)
                continue

            resp = future.result()
            if not resp.success:
                self.get_logger().error(
                    f'GenerateTrajectory rejected: {resp.message}')
                return None

            n_pts = len(resp.trajectory.points)
            self.get_logger().info(f'Trajectory generated: {n_pts} points. {resp.message}')
            self._publish_trajectory_markers(resp.trajectory)
            return resp.trajectory

        self.get_logger().error('GenerateTrajectory service exhausted all retries')
        self._publish_status('ERROR: GenerateTrajectory unavailable')
        return None

    # ── 4. Call ExecuteTrajectory action ────────────────────────────

    def _call_execute(self, trajectory):
        """Send trajectory to the action server and monitor progress."""
        if not self.exec_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action server unavailable')
            self._publish_status('ERROR: action server unavailable')
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f'Sending ExecuteTrajectory goal ({len(trajectory.points)} points)...')

        send_goal_future = self.exec_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal was rejected')
            self._publish_status('ERROR: goal rejected')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted — robot is moving!')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('ExecuteTrajectory timed out')
            self._publish_status('ERROR: execution timeout')
            return

        r = result.result
        if r.success:
            self.get_logger().info(
                f'Mission complete! '
                f'Duration: {r.total_time:.1f} s, '
                f'Max CTE: {r.max_cross_track_error:.4f} m, '
                f'Mean CTE: {r.mean_cross_track_error:.4f} m')
            self._publish_status(
                f'COMPLETE: {r.total_time:.1f}s, '
                f'maxCTE={r.max_cross_track_error:.4f}m')
        else:
            self.get_logger().warn('Trajectory execution was canceled or failed')
            self._publish_status('FAILED')

        self._goal_handle = None

    def _feedback_callback(self, feedback_msg):
        """Log progress from the action server."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {fb.progress_percent:.1f}%, '
            f'CTE: {fb.cross_track_error:.4f} m, '
            f'Heading err: {math.degrees(fb.heading_error):.1f}°',
            throttle_duration_sec=2.0)

    # ════════════════════════════════════════════════════════════════
    #  Visualization
    # ════════════════════════════════════════════════════════════════

    def _publish_waypoint_markers(self, waypoints):
        """Publish sphere markers for original waypoints (red)."""
        markers = MarkerArray()
        # Delete old markers first
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for i, wp in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints'
            m.id = i + 1  # start from 1 (0 is delete_all)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=0.12, y=0.12, z=0.12)
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            m.lifetime.sec = 0  # forever
            markers.markers.append(m)

            # Text label above sphere
            t = Marker()
            t.header = m.header
            t.ns = 'waypoint_labels'
            t.id = i + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp.x
            t.pose.position.y = wp.y
            t.pose.position.z = 0.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.08
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            t.text = f'WP{i}'
            markers.markers.append(t)

        self.waypoint_marker_pub.publish(markers)

    def _publish_trajectory_markers(self, trajectory):
        """Publish line strip + velocity-coloured markers for the trajectory."""
        markers = MarkerArray()

        # Line strip for trajectory path (green)
        line = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'trajectory_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.02  # line width

        max_v = max((pt.velocity for pt in trajectory.points), default=0.18)
        if max_v < 1e-6:
            max_v = 0.18

        for pt in trajectory.points:
            p = Point()
            p.x = pt.x
            p.y = pt.y
            p.z = 0.01
            line.points.append(p)

            # Colour: blue (slow) → green (medium) → red (fast)
            ratio = pt.velocity / max_v
            c = ColorRGBA()
            if ratio < 0.5:
                c.r = 0.0
                c.g = ratio * 2.0
                c.b = 1.0 - ratio * 2.0
            else:
                c.r = (ratio - 0.5) * 2.0
                c.g = 1.0 - (ratio - 0.5) * 2.0
                c.b = 0.0
            c.a = 0.9
            line.colors.append(c)

        markers.markers.append(line)

        # Velocity arrows at sampled points
        step = max(1, len(trajectory.points) // 20)
        for i in range(0, len(trajectory.points), step):
            pt = trajectory.points[i]
            arrow = Marker()
            arrow.header.frame_id = 'odom'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'velocity_arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            start = Point(x=pt.x, y=pt.y, z=0.02)
            length = pt.velocity * 0.5  # scale for visibility
            end = Point(
                x=pt.x + length * math.cos(pt.heading),
                y=pt.y + length * math.sin(pt.heading),
                z=0.02)
            arrow.points = [start, end]
            arrow.scale = Vector3(x=0.015, y=0.03, z=0.03)
            arrow.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.7)
            markers.markers.append(arrow)

        self.traj_marker_pub.publish(markers)

    # ════════════════════════════════════════════════════════════════
    #  Helpers
    # ════════════════════════════════════════════════════════════════

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Pipeline status: {status}')

    def _abort_callback(self, msg):
        if msg.data:
            self.get_logger().warn('ABORT requested!')
            self._abort_requested = True
            if self._goal_handle is not None:
                self.get_logger().info('Canceling active goal...')
                self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
