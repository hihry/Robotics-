#!/usr/bin/env python3
"""
plot_trajectory.py — Real-time and post-hoc trajectory visualization.

Subscribes to:
  /smoothed_path    (nav_msgs/Path)    — green
  /actual_path      (nav_msgs/Path)    — blue
  /tracking_error   (std_msgs/Float64) — tracking error over time

Usage inside Docker:
  ros2 run trajectory_control plot_trajectory.py
  # or standalone:
  python3 scripts/plot_trajectory.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64

import matplotlib
matplotlib.use('TkAgg')  # Use Tk backend for X11 forwarding
import matplotlib.pyplot as plt
import numpy as np
from collections import deque


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.smooth_path = None
        self.actual_xs = deque(maxlen=5000)
        self.actual_ys = deque(maxlen=5000)
        self.errors = deque(maxlen=5000)
        self.error_times = deque(maxlen=5000)
        self.t0 = None

        # Subscribers
        self.create_subscription(Path, '/smoothed_path', self.smooth_cb, 10)
        self.create_subscription(Path, '/actual_path', self.actual_cb, 10)
        self.create_subscription(Float64, '/tracking_error', self.error_cb, 10)

        # Setup matplotlib
        plt.ion()
        self.fig, (self.ax_path, self.ax_err) = plt.subplots(1, 2, figsize=(14, 6))
        self.fig.suptitle('Trajectory Tracking Performance')

        self.ax_path.set_xlabel('X (m)')
        self.ax_path.set_ylabel('Y (m)')
        self.ax_path.set_title('Path: Smoothed (green) vs Actual (blue)')
        self.ax_path.set_aspect('equal')
        self.ax_path.grid(True, alpha=0.3)

        self.ax_err.set_xlabel('Time (s)')
        self.ax_err.set_ylabel('Cross-Track Error (m)')
        self.ax_err.set_title('Tracking Error Over Time')
        self.ax_err.grid(True, alpha=0.3)

        # Timer for plot updates
        self.create_timer(0.5, self.update_plot)

        self.get_logger().info('Trajectory plotter started. Waiting for data...')

    def smooth_cb(self, msg: Path):
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        self.smooth_path = (xs, ys)

    def actual_cb(self, msg: Path):
        self.actual_xs.clear()
        self.actual_ys.clear()
        for p in msg.poses:
            self.actual_xs.append(p.pose.position.x)
            self.actual_ys.append(p.pose.position.y)

    def error_cb(self, msg: Float64):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        self.errors.append(msg.data)
        self.error_times.append(now - self.t0)

    def update_plot(self):
        # Path plot
        self.ax_path.clear()
        self.ax_path.set_xlabel('X (m)')
        self.ax_path.set_ylabel('Y (m)')
        self.ax_path.set_title('Path: Smoothed (green) vs Actual (blue)')
        self.ax_path.set_aspect('equal')
        self.ax_path.grid(True, alpha=0.3)

        if self.smooth_path:
            self.ax_path.plot(self.smooth_path[0], self.smooth_path[1],
                            'g-', linewidth=2, label='Smoothed')
        if self.actual_xs:
            self.ax_path.plot(list(self.actual_xs), list(self.actual_ys),
                            'b-', linewidth=1.5, alpha=0.8, label='Actual')
        self.ax_path.legend()

        # Error plot
        self.ax_err.clear()
        self.ax_err.set_xlabel('Time (s)')
        self.ax_err.set_ylabel('Cross-Track Error (m)')
        self.ax_err.set_title('Tracking Error Over Time')
        self.ax_err.grid(True, alpha=0.3)

        if self.errors:
            self.ax_err.plot(list(self.error_times), list(self.errors),
                           'r-', linewidth=1)
            avg_err = np.mean(list(self.errors))
            self.ax_err.axhline(y=avg_err, color='orange', linestyle='--',
                              label=f'Mean: {avg_err:.4f} m')
            self.ax_err.legend()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save final plot
        node.fig.savefig('/ros2_ws/trajectory_result.png', dpi=150, bbox_inches='tight')
        node.get_logger().info('Plot saved to /ros2_ws/trajectory_result.png')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
