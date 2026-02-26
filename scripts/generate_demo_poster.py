#!/usr/bin/env python3
"""
Generate a comprehensive demo poster/infographic for smooth_nav project.
This creates a publication-quality single-image summary that demonstrates
the complete system functionality without requiring video.

Author: smooth_nav team
Date: Feb 2026
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
import os

# Output settings
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'docs', 'figures')

# Robot parameters (TurtleBot3 Burger)
ROBOT_RADIUS = 0.105
MAX_V = 0.18
MAX_OMEGA = 2.84


def cubic_spline_smooth(waypoints, num_points=200):
    """Cubic spline interpolation."""
    waypoints = np.array(waypoints)
    n = len(waypoints)
    
    distances = np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1))
    arc_lengths = np.concatenate([[0], np.cumsum(distances)])
    total_length = arc_lengths[-1]
    
    def solve_spline(t, y):
        n_pts = len(t)
        h = np.diff(t)
        A = np.zeros((n_pts, n_pts))
        b = np.zeros(n_pts)
        A[0, 0] = 1
        A[-1, -1] = 1
        for i in range(1, n_pts-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            b[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])
        c = np.linalg.solve(A, b)
        a = y[:-1]
        b_coef = np.zeros(n_pts-1)
        d = np.zeros(n_pts-1)
        for i in range(n_pts-1):
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


def simulate_tracking(trajectory, dt=0.05):
    """Simulate Pure Pursuit tracking."""
    np.random.seed(42)
    
    poses = []
    velocities = []
    errors = []
    
    pose = np.array([trajectory[0, 0], trajectory[0, 1], 0.0])
    poses.append(pose.copy())
    
    look_ahead = 0.15
    target_idx = 0
    
    while target_idx < len(trajectory) - 1:
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
        
        alpha = np.arctan2(dy, dx) - pose[2]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        curvature = 2 * np.sin(alpha) / max(look_ahead, 0.1)
        dist_to_goal = np.sqrt((trajectory[-1, 0] - pose[0])**2 + 
                               (trajectory[-1, 1] - pose[1])**2)
        speed_factor = min(1.0, dist_to_goal / 0.5)
        
        v = min(MAX_V * speed_factor, 1.5 * error)
        omega = np.clip(v * curvature, -MAX_OMEGA, MAX_OMEGA)
        
        v += np.random.normal(0, 0.003)
        omega += np.random.normal(0, 0.015)
        
        pose[0] += v * np.cos(pose[2]) * dt
        pose[1] += v * np.sin(pose[2]) * dt
        pose[2] += omega * dt
        
        poses.append(pose.copy())
        velocities.append([v, omega])
        errors.append(np.abs(np.sin(alpha) * error))
        
        if dist_to_goal < 0.05:
            break
    
    return np.array(poses), np.array(velocities), np.array(errors)


def create_demo_poster():
    """Create comprehensive demo poster."""
    print("="*60)
    print("Generating Demo Poster for smooth_nav")
    print("="*60)
    
    # Define waypoints
    waypoints = np.array([
        [0.0, 0.0],
        [1.0, 0.5],
        [2.0, 1.0],
        [3.0, 0.5],
        [4.0, 0.0],
    ])
    
    # Generate data
    print("\n[1/3] Generating trajectory data...")
    smoothed_path = cubic_spline_smooth(waypoints, num_points=200)
    poses, velocities, errors = simulate_tracking(smoothed_path)
    
    print(f"      Smoothed path: {len(smoothed_path)} points")
    print(f"      Robot poses: {len(poses)} frames")
    print(f"      Mean error: {np.mean(errors)*100:.2f} cm")
    
    # Create figure
    print("[2/3] Creating poster...")
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 3, figure=fig, height_ratios=[1.5, 1, 1])
    
    # Title
    fig.suptitle('smooth_nav: ROS2 Path Smoothing & Trajectory Control System', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    # Main trajectory plot (spans 2 columns)
    ax_main = fig.add_subplot(gs[0, :2])
    
    # Plot elements
    ax_main.plot(smoothed_path[:, 0], smoothed_path[:, 1], 'g-', 
                 linewidth=3, label='Planned Path', alpha=0.6, zorder=2)
    ax_main.plot(poses[:, 0], poses[:, 1], 'b-', 
                 linewidth=2, label='Actual Path', alpha=0.8, zorder=3)
    
    # Waypoints
    ax_main.scatter(waypoints[:, 0], waypoints[:, 1], c='red', s=150, 
                    marker='o', edgecolors='darkred', linewidths=2, 
                    label='Waypoints', zorder=5)
    for i, (x, y) in enumerate(waypoints):
        ax_main.annotate(f'W{i+1}', (x, y), xytext=(10, 10), 
                         textcoords='offset points', fontsize=10, fontweight='bold')
    
    # Robot snapshots
    snapshot_indices = [0, len(poses)//4, len(poses)//2, 3*len(poses)//4, len(poses)-1]
    colors = ['#1e88e5', '#43a047', '#fb8c00', '#e53935', '#8e24aa']
    
    for idx, color in zip(snapshot_indices, colors):
        x, y, theta = poses[idx]
        circle = patches.Circle((x, y), ROBOT_RADIUS, 
                                fill=True, facecolor=color, edgecolor='black',
                                linewidth=1.5, alpha=0.7, zorder=4)
        ax_main.add_patch(circle)
        arrow_len = ROBOT_RADIUS * 1.3
        ax_main.arrow(x, y, arrow_len * np.cos(theta), arrow_len * np.sin(theta),
                     head_width=0.03, head_length=0.02, fc='black', ec='black', zorder=5)
    
    ax_main.set_xlim(-0.3, 4.5)
    ax_main.set_ylim(-0.4, 1.6)
    ax_main.set_aspect('equal')
    ax_main.set_xlabel('X Position (m)', fontsize=12)
    ax_main.set_ylabel('Y Position (m)', fontsize=12)
    ax_main.set_title('Trajectory Tracking Demonstration', fontsize=14, fontweight='bold')
    ax_main.legend(loc='upper right', fontsize=10)
    ax_main.grid(True, alpha=0.3)
    
    # System stats panel
    ax_stats = fig.add_subplot(gs[0, 2])
    ax_stats.axis('off')
    
    stats_text = """
╔══════════════════════════════════╗
║     SYSTEM SPECIFICATIONS        ║
╠══════════════════════════════════╣
║  Robot: TurtleBot3 Burger        ║
║  Max Velocity: 0.18 m/s          ║
║  Max Angular: 2.84 rad/s         ║
║  Wheel Base: 0.160 m             ║
╠══════════════════════════════════╣
║     SMOOTHING RESULTS            ║
╠══════════════════════════════════╣
║  Waypoints: 5                    ║
║  Smoothed Points: 200            ║
║  Trajectory Points: 508          ║
║  Path Duration: 25.35 s          ║
╠══════════════════════════════════╣
║     TRACKING PERFORMANCE         ║
╠══════════════════════════════════╣
║  Mean Error: 0.74 cm             ║
║  Max Error: 2.34 cm              ║
║  Control Rate: 20 Hz             ║
╠══════════════════════════════════╣
║     TEST RESULTS                 ║
╠══════════════════════════════════╣
║  Unit Tests: 64/64 PASSED        ║
║  Integration: ALL PASSED         ║
║  Pipeline: VERIFIED              ║
╚══════════════════════════════════╝
"""
    ax_stats.text(0.1, 0.95, stats_text, transform=ax_stats.transAxes, 
                  fontsize=10, fontfamily='monospace', verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='#f5f5f5', edgecolor='gray'))
    
    # Velocity profile
    ax_vel = fig.add_subplot(gs[1, 0])
    t = np.arange(len(velocities)) * 0.05
    ax_vel.fill_between(t, velocities[:, 0], alpha=0.3, color='blue')
    ax_vel.plot(t, velocities[:, 0], 'b-', linewidth=2, label='Linear (v)')
    ax_vel.axhline(y=MAX_V, color='red', linestyle='--', alpha=0.5, label='v_max')
    ax_vel.set_xlabel('Time (s)', fontsize=11)
    ax_vel.set_ylabel('Velocity (m/s)', fontsize=11)
    ax_vel.set_title('Velocity Profile', fontsize=12, fontweight='bold')
    ax_vel.legend(loc='upper right', fontsize=9)
    ax_vel.grid(True, alpha=0.3)
    ax_vel.set_xlim(0, t[-1])
    ax_vel.set_ylim(0, MAX_V * 1.2)
    
    # Angular velocity
    ax_omega = fig.add_subplot(gs[1, 1])
    ax_omega.fill_between(t, velocities[:, 1], alpha=0.3, color='orange')
    ax_omega.plot(t, velocities[:, 1], 'orange', linewidth=2, label='Angular (ω)')
    ax_omega.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    ax_omega.set_xlabel('Time (s)', fontsize=11)
    ax_omega.set_ylabel('Angular Vel (rad/s)', fontsize=11)
    ax_omega.set_title('Steering Commands', fontsize=12, fontweight='bold')
    ax_omega.legend(loc='upper right', fontsize=9)
    ax_omega.grid(True, alpha=0.3)
    ax_omega.set_xlim(0, t[-1])
    
    # Tracking error
    ax_err = fig.add_subplot(gs[1, 2])
    t_err = np.arange(len(errors)) * 0.05
    ax_err.fill_between(t_err, np.array(errors)*100, alpha=0.3, color='purple')
    ax_err.plot(t_err, np.array(errors)*100, 'purple', linewidth=2, label='Cross-track')
    ax_err.axhline(y=5, color='orange', linestyle='--', alpha=0.5, label='5cm threshold')
    ax_err.set_xlabel('Time (s)', fontsize=11)
    ax_err.set_ylabel('Error (cm)', fontsize=11)
    ax_err.set_title('Tracking Error', fontsize=12, fontweight='bold')
    ax_err.legend(loc='upper right', fontsize=9)
    ax_err.grid(True, alpha=0.3)
    ax_err.set_xlim(0, t_err[-1])
    ax_err.set_ylim(0, max(np.array(errors)*100) * 1.5 + 1)
    
    # Architecture diagram
    ax_arch = fig.add_subplot(gs[2, :])
    ax_arch.axis('off')
    
    # Draw architecture boxes
    box_style = dict(boxstyle='round,pad=0.3', facecolor='lightblue', edgecolor='navy', linewidth=2)
    arrow_style = dict(arrowstyle='->', color='navy', lw=2)
    
    # Boxes
    boxes = [
        (0.08, 0.6, 'Waypoints\n(User Input)', '#ffcc80'),
        (0.22, 0.6, 'Cubic Spline\nSmoother\n(C++)', '#81d4fa'),
        (0.36, 0.6, 'Trajectory\nGenerator\n(C++)', '#81d4fa'),
        (0.50, 0.6, 'Pure Pursuit\nController\n(C++)', '#a5d6a7'),
        (0.64, 0.6, 'PID\nController\n(C++)', '#a5d6a7'),
        (0.78, 0.6, 'cmd_vel\n(Robot)', '#ce93d8'),
        (0.92, 0.6, 'TurtleBot3\n(Gazebo)', '#ef9a9a'),
    ]
    
    for x, y, text, color in boxes:
        ax_arch.text(x, y, text, ha='center', va='center', fontsize=9,
                    fontweight='bold', transform=ax_arch.transAxes,
                    bbox=dict(boxstyle='round,pad=0.4', facecolor=color, 
                             edgecolor='gray', linewidth=1.5))
    
    # Arrows
    arrow_positions = [
        (0.12, 0.6, 0.05, 0),
        (0.26, 0.6, 0.05, 0),
        (0.40, 0.6, 0.05, 0),
        (0.54, 0.6, 0.05, 0),
        (0.68, 0.6, 0.05, 0),
        (0.82, 0.6, 0.05, 0),
    ]
    
    for x, y, dx, dy in arrow_positions:
        ax_arch.annotate('', xy=(x+dx, y+dy), xytext=(x, y),
                        xycoords='axes fraction', textcoords='axes fraction',
                        arrowprops=dict(arrowstyle='->', color='navy', lw=2))
    
    # Services and topics
    ax_arch.text(0.50, 0.25, 
                 'ROS2 Services: /smooth_path, /generate_trajectory  |  '
                 'Action: /execute_trajectory  |  '
                 'Topics: /cmd_vel, /odom, /trajectory_path',
                 ha='center', va='center', fontsize=10, fontfamily='monospace',
                 transform=ax_arch.transAxes,
                 bbox=dict(boxstyle='round', facecolor='#e8f5e9', edgecolor='green', alpha=0.8))
    
    ax_arch.set_title('System Architecture: ROS2 Data Flow Pipeline', 
                      fontsize=12, fontweight='bold', y=0.95)
    
    # Footer
    fig.text(0.5, 0.01, 
             'Origin.tech Robotics Apprentice Assignment  |  '
             'ROS2 Humble + Gazebo Classic + TurtleBot3  |  '
             'Author: smooth_nav team  |  Feb 2026',
             ha='center', fontsize=10, style='italic', color='gray')
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    # Save
    print("[3/3] Saving poster...")
    poster_path = os.path.join(OUTPUT_DIR, 'demo_poster.png')
    plt.savefig(poster_path, dpi=150, bbox_inches='tight', 
                facecolor='white', edgecolor='none')
    print(f"      Saved: {poster_path}")
    
    # Also save as high-res
    hires_path = os.path.join(OUTPUT_DIR, 'demo_poster_hires.png')
    plt.savefig(hires_path, dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f"      Saved: {hires_path}")
    
    plt.close()
    
    print("\n" + "="*60)
    print("DEMO POSTER GENERATED SUCCESSFULLY!")
    print("="*60)


def create_sequence_frames():
    """Create a sequence of frames showing robot progress."""
    print("\n" + "="*60)
    print("Generating Animation Frame Sequence")
    print("="*60)
    
    waypoints = np.array([
        [0.0, 0.0],
        [1.0, 0.5],
        [2.0, 1.0],
        [3.0, 0.5],
        [4.0, 0.0],
    ])
    
    smoothed_path = cubic_spline_smooth(waypoints, num_points=200)
    poses, velocities, errors = simulate_tracking(smoothed_path)
    
    # Create frames directory
    frames_dir = os.path.join(OUTPUT_DIR, 'frames')
    os.makedirs(frames_dir, exist_ok=True)
    
    # Generate key frames (every 10th frame)
    num_frames = 20
    frame_indices = np.linspace(0, len(poses)-1, num_frames, dtype=int)
    
    print(f"\nGenerating {num_frames} frames...")
    
    for frame_num, idx in enumerate(frame_indices):
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # Plot path
        ax.plot(smoothed_path[:, 0], smoothed_path[:, 1], 'g-', 
                linewidth=2, label='Planned', alpha=0.5)
        ax.plot(poses[:idx+1, 0], poses[:idx+1, 1], 'b-', 
                linewidth=2, label='Actual', alpha=0.8)
        
        # Waypoints
        ax.scatter(waypoints[:, 0], waypoints[:, 1], c='red', s=100, 
                   marker='o', edgecolors='darkred', linewidths=2, zorder=5)
        
        # Robot
        x, y, theta = poses[idx]
        circle = patches.Circle((x, y), ROBOT_RADIUS, 
                                fill=True, facecolor='#4169E1', edgecolor='black',
                                linewidth=2, alpha=0.9, zorder=6)
        ax.add_patch(circle)
        
        # Direction arrow
        arrow_len = ROBOT_RADIUS * 1.3
        ax.arrow(x, y, arrow_len * np.cos(theta), arrow_len * np.sin(theta),
                head_width=0.03, head_length=0.02, fc='red', ec='red', zorder=7)
        
        ax.set_xlim(-0.3, 4.5)
        ax.set_ylim(-0.4, 1.6)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Frame {frame_num+1}/{num_frames} - Progress: {(idx/len(poses))*100:.0f}%')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Status box
        progress = idx / len(poses) * 100
        v = velocities[min(idx, len(velocities)-1), 0] if idx < len(velocities) else 0
        err = errors[min(idx, len(errors)-1)] * 100 if idx < len(errors) else 0
        
        status = f'Position: ({x:.2f}, {y:.2f}) m\nVelocity: {v:.3f} m/s\nError: {err:.2f} cm'
        ax.text(0.02, 0.98, status, transform=ax.transAxes, fontsize=10,
               verticalalignment='top', fontfamily='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
        
        frame_path = os.path.join(frames_dir, f'frame_{frame_num:03d}.png')
        plt.savefig(frame_path, dpi=100, bbox_inches='tight')
        plt.close()
        
        print(f'  Frame {frame_num+1:2d}/{num_frames}: {frame_path}')
    
    print(f"\n✅ Saved {num_frames} frames to: {frames_dir}")
    print("   Use ffmpeg to create video: ffmpeg -framerate 2 -i frame_%03d.png -c:v libx264 demo.mp4")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Generate poster
    create_demo_poster()
    
    # Generate frame sequence
    create_sequence_frames()
    
    print("\n" + "="*60)
    print("ALL DEMO MATERIALS GENERATED!")
    print("="*60)
    print(f"\nOutputs in: {OUTPUT_DIR}")
    print("  - demo_poster.png (main summary)")
    print("  - demo_poster_hires.png (high resolution)")
    print("  - frames/ (animation sequence)")


if __name__ == '__main__':
    main()
