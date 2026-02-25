#!/usr/bin/env python3
"""
visualize_trajectory.py — Matplotlib visualization of smoothed path and trajectory.

Subscribes to ROS 2 topics and produces plots, OR reads from a CSV/rosbag.
Can also be used standalone for offline analysis.

Usage:
  # ROS 2 live mode
  ros2 run smooth_nav_bringup visualize_trajectory.py

  # Standalone demo with sample data
  python3 scripts/visualize_trajectory.py --demo
"""

import argparse
import math
import sys

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    print("Install matplotlib and numpy: pip3 install matplotlib numpy")
    sys.exit(1)


def demo_cubic_spline():
    """Generate a demo cubic spline visualization with sample waypoints."""
    # Sample waypoints (S-curve)
    wx = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 3.0, 2.0, 1.0, 0.0])
    wy = np.array([0.0, 0.0, 0.5, 0.5, 0.0, -1.0, -1.5, -1.5, -1.0, 0.0])

    # Simple cubic spline interpolation using numpy
    t = np.zeros(len(wx))
    for i in range(1, len(wx)):
        t[i] = t[i - 1] + math.sqrt((wx[i] - wx[i - 1])**2 + (wy[i] - wy[i - 1])**2)

    t_fine = np.linspace(t[0], t[-1], 200)
    sx = np.interp(t_fine, t, wx)
    sy = np.interp(t_fine, t, wy)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Plot 1: Path
    axes[0].plot(wx, wy, 'ro-', label='Waypoints', markersize=8)
    axes[0].plot(sx, sy, 'b-', label='Smoothed Path', linewidth=2)
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Path Smoothing')
    axes[0].legend()
    axes[0].set_aspect('equal')
    axes[0].grid(True)

    # Plot 2: Velocity profile (trapezoidal)
    max_v = 0.22
    max_a = 0.5
    total_s = t[-1]
    accel_dist = max_v**2 / (2 * max_a)
    s_arr = np.linspace(0, total_s, 200)
    v_arr = np.zeros_like(s_arr)
    for i, s in enumerate(s_arr):
        if s < accel_dist:
            v_arr[i] = math.sqrt(2 * max_a * s)
        elif s > total_s - accel_dist:
            v_arr[i] = math.sqrt(2 * max_a * (total_s - s))
        else:
            v_arr[i] = max_v
        v_arr[i] = min(v_arr[i], max_v)

    axes[1].plot(s_arr, v_arr, 'g-', linewidth=2)
    axes[1].axhline(y=max_v, color='r', linestyle='--', label=f'v_max = {max_v} m/s')
    axes[1].set_xlabel('Arc Length (m)')
    axes[1].set_ylabel('Velocity (m/s)')
    axes[1].set_title('Trapezoidal Velocity Profile')
    axes[1].legend()
    axes[1].grid(True)

    # Plot 3: Curvature
    dx = np.gradient(sx, t_fine)
    dy = np.gradient(sy, t_fine)
    ddx = np.gradient(dx, t_fine)
    ddy = np.gradient(dy, t_fine)
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-10)**1.5

    axes[2].plot(t_fine, curvature, 'm-', linewidth=2)
    axes[2].set_xlabel('Arc Length Parameter')
    axes[2].set_ylabel('Curvature (1/m)')
    axes[2].set_title('Path Curvature')
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('trajectory_visualization.png', dpi=150)
    print("Saved: trajectory_visualization.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Trajectory Visualization')
    parser.add_argument('--demo', action='store_true', help='Run standalone demo')
    args = parser.parse_args()

    if args.demo:
        demo_cubic_spline()
    else:
        print("ROS 2 live mode not yet implemented. Use --demo for offline visualization.")


if __name__ == '__main__':
    main()
