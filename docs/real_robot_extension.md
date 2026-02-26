# Real Robot Extension — smooth_nav

This document outlines how the `smooth_nav` system can be extended to run on a physical TurtleBot3 Burger.

---

## Overview

The architecture is designed for a smooth simulation-to-real transition:

1. `smooth_nav_core` — **No changes needed** (pure math, no hardware coupling)
2. `smooth_nav_ros` — **No changes needed** (services are robot-agnostic)
3. `smooth_nav_controller` — **Minimal changes** (tune PID gains for real dynamics)
4. `smooth_nav_bringup` — **New launch file** for real robot

---

## Steps

### 1. Network Setup
```bash
# On the robot (Raspberry Pi)
export ROS_DOMAIN_ID=30

# On the workstation
export ROS_DOMAIN_ID=30
```

### 2. Launch on Robot
```bash
# SSH into TurtleBot3
ros2 launch turtlebot3_bringup robot.launch.py
```

### 3. Launch smooth_nav on Workstation
```bash
# Only the service nodes + controller (no Gazebo)
ros2 launch smooth_nav_ros smooth_nav_ros.launch.py
ros2 launch smooth_nav_controller controller.launch.py
```

### 4. Tuning

Key parameters to adjust for real hardware (all live-tunable via `rqt_reconfigure`):

| Parameter | Sim Value | Real Starting Value | Notes |
|-----------|-----------|-------------------|-------|
| `max_linear_velocity` | 0.22 | 0.15 | Start conservative |
| `max_angular_velocity` | 2.84 | 1.5 | Reduce for stability |
| `look_ahead_distance` | 0.3 | 0.4 | Increase for sensor lag |
| `adaptive_look_ahead_gain` | 0.5 | 0.3 | Lower for real-world lag |
| `max_lateral_acceleration` | 1.0 | 0.5 | Tighter for wheel-slip prevention |
| `goal_deceleration_radius` | 0.5 | 0.8 | Larger for real inertia |
| `kp_heading` | 2.0 | 1.0 | Lower gains initially |
| `goal_tolerance` | 0.05 | 0.10 | Larger for real noise |
| `control_rate` | 20.0 | 10.0 | Match sensor rate |

### 5. Safety Layer (Already Implemented)

The following safety features are **already part of smooth_nav** via `safety_watchdog_node`:

| Feature | Status | Details |
|---------|--------|---------|
| **Emergency stop** (laser proximity) | ✅ Implemented | Subscribes to `/scan`, stops when obstacle within `safety_distance` (default 0.3 m) |
| **Velocity governor** | ✅ Implemented | Clamps `/cmd_vel_raw` to `max_linear_velocity` / `max_angular_velocity` |
| **Command timeout watchdog** | ✅ Implemented | Publishes zero-velocity if no command received within `cmd_vel_timeout` (default 0.5 s) |
| **Acceleration limiter** | ✅ Implemented | Limits linear acceleration to `max_linear_acceleration` (default 0.5 m/s²) |

For real deployment, tune the safety parameters:
```bash
ros2 param set /safety_watchdog safety_distance 0.5        # wider margin
ros2 param set /safety_watchdog max_linear_velocity 0.15   # conservative
ros2 param set /safety_watchdog cmd_vel_timeout 0.3        # faster timeout
```

---

## What Would Change

| Component | Change |
|-----------|--------|
| `smooth_nav_controller` config | Tune PID gains, velocities |
| `smooth_nav_bringup` | New `real_robot.launch.py` |
| `smooth_nav_simulation` | Not used on real robot |
| URDF | Use official `turtlebot3_description` directly |

---

## Testing on Real Hardware

1. Start with a 1 m straight line at 0.1 m/s
2. Measure cross-track error with a measuring tape
3. Gradually increase speed and path complexity
4. Record rosbag for offline analysis: `./scripts/record_bag.sh`
