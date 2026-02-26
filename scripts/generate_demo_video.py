#!/usr/bin/env python3
"""
Generate animated demonstration video for smooth_nav project.
Creates an MP4 showing the robot following the trajectory.

This is a headless alternative that doesn't require X Server or Gazebo GUI.

Author: smooth_nav team
Date: Feb 2026
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
import os

# Output settings
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'docs', 'figures')
VIDEO_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'demo_video.mp4')

# Robot parameters (TurtleBot3 Burger)
ROBOT_RADIUS = 0.105  # meters
WHEEL_BASE = 0.160
MAX_V = 0.18  # m/s
MAX_OMEGA = 2.84  # rad/s


def cubic_spline_smooth(waypoints, num_points=200):
    """Cubic spline interpolation."""
    waypoints = np.array(waypoints)
    n = len(waypoints)
    
    distances = np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1))
    arc_lengths = np.concatenate([[0], np.cumsum(distances)])
    total_length = arc_lengths[-1]
    
    def solve_spline(t, y):
        n = len(t)
        h = np.diff(t)
        A = np.zeros((n, n))
        b = np.zeros(n)
        A[0, 0] = 1
        A[-1, -1] = 1
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            b[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])
        c = np.linalg.solve(A, b)
        a = y[:-1]
        b_coef = np.zeros(n-1)
        d = np.zeros(n-1)
        for i in range(n-1):
            b_coef[i] = (y[i+1] - y[i]) / h[i] - h[i] * (2*c[i] + c[i+1]) / 3
            d[i] = (c[i+1] - c[i]) / (3 * h[i])
        return a, b_coef, c[:-1], d, t[:-1], h
    
    ax, bx, cx, dx, tx, hx = solve_spline(arc_lengths, waypoints[:, 0])
    ay, by, cy, dy, ty, hy = solve_spline(arc_lengths, waypoints[:, 1])
    
    s_samples = np.linspace(0, total_length, num_points)
    smoothed = []
    
    for s in s_samples:
        idx = min(np.searchsorted(arc_lengths, s) - 1, n - 2)
        idx = max(0, idx)
        ds = s - arc_lengths[idx]
        x = ax[idx] + bx[idx]*ds + cx[idx]*ds**2 + dx[idx]*ds**3
        y = ay[idx] + by[idx]*ds + cy[idx]*ds**2 + dy[idx]*ds**3
        smoothed.append([x, y])
    
    return np.array(smoothed)


def simulate_robot_motion(trajectory, dt=0.05):
    """Simulate Pure Pursuit + PID controller."""
    np.random.seed(42)
    
    poses = []  # (x, y, theta)
    velocities = []  # (v, omega)
    tracking_errors = []
    
    pose = np.array([trajectory[0, 0], trajectory[0, 1], 0.0])
    poses.append(pose.copy())
    
    look_ahead = 0.15
    k_v = 1.5
    
    target_idx = 0
    
    while target_idx < len(trajectory) - 1:
        # Find look-ahead point
        while target_idx < len(trajectory) - 1:
            dist = np.sqrt((trajectory[target_idx, 0] - pose[0])**2 + 
                          (trajectory[target_idx, 1] - pose[1])**2)
            if dist >= look_ahead:
                break
            target_idx += 1
        
        target = trajectory[min(target_idx + 3, len(trajectory)-1)]
        
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        error = np.sqrt(dx**2 + dy**2)
        
        # Pure pursuit
        alpha = np.arctan2(dy, dx) - pose[2]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        curvature = 2 * np.sin(alpha) / max(look_ahead, 0.1)
        
        # Goal deceleration
        dist_to_goal = np.sqrt((trajectory[-1, 0] - pose[0])**2 + 
                               (trajectory[-1, 1] - pose[1])**2)
        speed_factor = min(1.0, dist_to_goal / 0.5)
        
        v = min(MAX_V * speed_factor, k_v * error)
        omega = v * curvature
        omega = np.clip(omega, -MAX_OMEGA, MAX_OMEGA)
        
        # Add noise
        v += np.random.normal(0, 0.003)
        omega += np.random.normal(0, 0.015)
        
        # Update pose
        pose[0] += v * np.cos(pose[2]) * dt
        pose[1] += v * np.sin(pose[2]) * dt
        pose[2] += omega * dt
        
        poses.append(pose.copy())
        velocities.append([v, omega])
        
        # Cross-track error
        cross_track = np.abs(np.sin(alpha) * error)
        tracking_errors.append(cross_track)
        
        if dist_to_goal < 0.05:
            break
    
    return np.array(poses), np.array(velocities), np.array(tracking_errors)


def draw_robot(ax, x, y, theta, color='blue'):
    """Draw TurtleBot3-like robot."""
    # Robot body (circle)
    circle = patches.Circle((x, y), ROBOT_RADIUS, 
                             fill=True, facecolor=color, edgecolor='black', 
                             linewidth=2, alpha=0.8, zorder=5)
    ax.add_patch(circle)
    
    # Direction indicator
    arrow_len = ROBOT_RADIUS * 1.2
    ax.arrow(x, y, arrow_len * np.cos(theta), arrow_len * np.sin(theta),
             head_width=0.03, head_length=0.02, fc='red', ec='red', zorder=6)
    
    return circle


def create_demo_animation():
    """Create the demo animation."""
    print("="*60)
    print("Generating Demo Animation for smooth_nav")
    print("="*60)
    
    # Define waypoints
    waypoints = np.array([
        [0.0, 0.0],
        [1.0, 0.5],
        [2.0, 1.0],
        [3.0, 0.5],
        [4.0, 0.0],
    ])
    
    # Generate smoothed path
    print("\n[1/4] Generating smoothed path...")
    smoothed_path = cubic_spline_smooth(waypoints, num_points=200)
    
    # Simulate robot motion
    print("[2/4] Simulating robot motion...")
    poses, velocities, errors = simulate_robot_motion(smoothed_path)
    
    print(f"      Generated {len(poses)} robot poses")
    print(f"      Mean tracking error: {np.mean(errors)*100:.2f} cm")
    
    # Create figure
    print("[3/4] Creating animation...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Main trajectory plot
    ax_main = axes[0, 0]
    ax_main.set_xlim(-0.5, 4.5)
    ax_main.set_ylim(-0.5, 1.8)
    ax_main.set_aspect('equal')
    ax_main.set_xlabel('X Position (m)', fontsize=12)
    ax_main.set_ylabel('Y Position (m)', fontsize=12)
    ax_main.set_title('Trajectory Tracking Demo', fontsize=14, fontweight='bold')
    ax_main.grid(True, alpha=0.3)
    
    # Plot static elements
    ax_main.plot(smoothed_path[:, 0], smoothed_path[:, 1], 'g-', linewidth=2, 
                 label='Planned Path', alpha=0.6)
    ax_main.scatter(waypoints[:, 0], waypoints[:, 1], c='red', s=100, marker='o',
                    edgecolors='darkred', linewidths=2, label='Waypoints', zorder=4)
    for i, (x, y) in enumerate(waypoints):
        ax_main.annotate(f'W{i+1}', (x, y), xytext=(8, 8), textcoords='offset points',
                         fontsize=9, fontweight='bold')
    ax_main.legend(loc='upper right')
    
    # Velocity plot
    ax_vel = axes[0, 1]
    ax_vel.set_xlim(0, len(velocities) * 0.05)
    ax_vel.set_ylim(0, MAX_V * 1.2)
    ax_vel.set_xlabel('Time (s)', fontsize=12)
    ax_vel.set_ylabel('Linear Velocity (m/s)', fontsize=12)
    ax_vel.set_title('Velocity Command', fontsize=14, fontweight='bold')
    ax_vel.grid(True, alpha=0.3)
    ax_vel.axhline(y=MAX_V, color='red', linestyle='--', alpha=0.5, label='v_max')
    ax_vel.legend(loc='upper right')
    
    # Angular velocity plot
    ax_omega = axes[1, 0]
    ax_omega.set_xlim(0, len(velocities) * 0.05)
    ax_omega.set_ylim(-MAX_OMEGA * 0.5, MAX_OMEGA * 0.5)
    ax_omega.set_xlabel('Time (s)', fontsize=12)
    ax_omega.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    ax_omega.set_title('Steering Command', fontsize=14, fontweight='bold')
    ax_omega.grid(True, alpha=0.3)
    ax_omega.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    
    # Tracking error plot
    ax_err = axes[1, 1]
    ax_err.set_xlim(0, len(errors) * 0.05)
    ax_err.set_ylim(0, max(errors) * 100 * 1.5 + 1)
    ax_err.set_xlabel('Time (s)', fontsize=12)
    ax_err.set_ylabel('Cross-Track Error (cm)', fontsize=12)
    ax_err.set_title('Tracking Error', fontsize=14, fontweight='bold')
    ax_err.grid(True, alpha=0.3)
    ax_err.axhline(y=5, color='orange', linestyle='--', alpha=0.5, label='5cm threshold')
    ax_err.legend(loc='upper right')
    
    # Animation elements
    robot_circle = None
    actual_path_line, = ax_main.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Actual Path')
    vel_line, = ax_vel.plot([], [], 'b-', linewidth=2)
    omega_line, = ax_omega.plot([], [], 'orange', linewidth=2)
    err_line, = ax_err.plot([], [], 'purple', linewidth=2)
    
    # Status text
    status_text = ax_main.text(0.02, 0.98, '', transform=ax_main.transAxes,
                               fontsize=11, verticalalignment='top',
                               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    
    def init():
        actual_path_line.set_data([], [])
        vel_line.set_data([], [])
        omega_line.set_data([], [])
        err_line.set_data([], [])
        status_text.set_text('')
        return actual_path_line, vel_line, omega_line, err_line, status_text
    
    def animate(frame):
        nonlocal robot_circle
        
        # Remove previous robot
        if robot_circle is not None:
            robot_circle.remove()
        
        # Current pose
        x, y, theta = poses[frame]
        
        # Draw robot
        robot_circle = patches.Circle((x, y), ROBOT_RADIUS, 
                                      fill=True, facecolor='#4169E1', 
                                      edgecolor='black', linewidth=2, 
                                      alpha=0.9, zorder=5)
        ax_main.add_patch(robot_circle)
        
        # Direction arrow
        arrow_len = ROBOT_RADIUS * 1.3
        ax_main.annotate('', xy=(x + arrow_len * np.cos(theta), 
                                  y + arrow_len * np.sin(theta)),
                         xytext=(x, y),
                         arrowprops=dict(arrowstyle='->', color='red', lw=2),
                         zorder=6)
        
        # Update actual path
        actual_path_line.set_data(poses[:frame+1, 0], poses[:frame+1, 1])
        
        # Update velocity plot
        t_data = np.arange(min(frame, len(velocities))) * 0.05
        if frame > 0 and frame <= len(velocities):
            vel_line.set_data(t_data, velocities[:frame, 0])
            omega_line.set_data(t_data, velocities[:frame, 1])
        
        # Update error plot
        if frame > 0 and frame <= len(errors):
            t_err = np.arange(min(frame, len(errors))) * 0.05
            err_line.set_data(t_err, np.array(errors[:frame]) * 100)
        
        # Progress
        progress = frame / len(poses) * 100
        
        # Status
        if frame < len(velocities):
            v, omega = velocities[frame]
        else:
            v, omega = 0, 0
        
        if frame < len(errors):
            err = errors[frame] * 100
        else:
            err = 0
        
        status_text.set_text(
            f'Progress: {progress:.1f}%\n'
            f'Position: ({x:.2f}, {y:.2f}) m\n'
            f'Velocity: {v:.3f} m/s\n'
            f'Error: {err:.2f} cm'
        )
        
        return actual_path_line, vel_line, omega_line, err_line, status_text
    
    # Create animation
    frames = len(poses)
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=frames, interval=50, blit=False)
    
    plt.tight_layout()
    
    # Save animation
    print("[4/4] Saving animation...")
    print(f"      Output: {VIDEO_PATH}")
    
    # Check for ffmpeg
    try:
        writer = animation.FFMpegWriter(fps=20, metadata=dict(
            title='smooth_nav Demo',
            artist='smooth_nav team',
            comment='ROS2 Path Smoothing & Trajectory Control Demo'
        ), bitrate=2000)
        anim.save(VIDEO_PATH, writer=writer)
        print(f"\n✅ Video saved: {VIDEO_PATH}")
        return True
    except Exception as e:
        print(f"\n⚠️ FFmpeg not available: {e}")
        print("   Saving as GIF instead...")
        gif_path = VIDEO_PATH.replace('.mp4', '.gif')
        try:
            anim.save(gif_path, writer='pillow', fps=15)
            print(f"✅ GIF saved: {gif_path}")
            return True
        except Exception as e2:
            print(f"❌ Could not save animation: {e2}")
            print("\n   Install ffmpeg: winget install ffmpeg")
            print("   Or: pip install imageio")
            return False


def main():
    success = create_demo_animation()
    
    if success:
        print("\n" + "="*60)
        print("DEMO VIDEO GENERATED SUCCESSFULLY!")
        print("="*60)
    else:
        print("\n" + "="*60)
        print("Animation could not be saved.")
        print("Install ffmpeg or imageio to create video/GIF.")
        print("="*60)


if __name__ == '__main__':
    main()
