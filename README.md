# TurtleBot3 Path Smoothing & Trajectory Control

ROS2 Humble package for **2D path smoothing** and **trajectory tracking** on TurtleBot3 Burger in Gazebo simulation.

## Assignment Overview

| Component | Points | Algorithm |
|-----------|--------|-----------|
| Path Smoothing | 25 | Cubic Spline Interpolation + Gradient Descent |
| Trajectory Generation | 25 | Trapezoidal Velocity Profile (arc-length parameterized) |
| Trajectory Tracking | 25 | Pure Pursuit + PID Cross-Track Correction |
| Code Quality & Testing | 15 | Google Test, Doxygen comments |
| Documentation & Demo | 10 | README, plots, screen recording |

---

## Quick Start (Docker)

### Prerequisites
- **Docker** ≥ 20.x (Docker Desktop on Windows)
- **VcXsrv** (Windows X Server for GUI — Gazebo/RViz2)

### 1. Build the Docker image
```bash
docker compose build ros2-nogpu
```

### 2. Start VcXsrv
Launch VcXsrv with these settings:
- Display number: **0**
- Start no client
- ✅ Disable access control

### 3. Run the container
```bash
docker compose run --rm ros2-nogpu
```

### 4. Inside the container — build & run
```bash
cd /ros2_ws
colcon build --packages-select trajectory_control
source install/setup.bash

# Launch full demo (Gazebo + controller + RViz2)
ros2 launch trajectory_control trajectory_demo.launch.py
```

### 5. Run tests
```bash
colcon test --packages-select trajectory_control
colcon test-result --verbose
```

### 6. Plot results (separate terminal)
```bash
# In another terminal attached to the same container:
docker exec -it <container_id> bash
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/trajectory_control/scripts/plot_trajectory.py
```

---

## Architecture

```
waypoints → [PathSmoother] → smooth_path → [TrajectoryGenerator] → trajectory
                                                                        ↓
                            /cmd_vel ← [TrajectoryController] ← /odom + trajectory
```

### PathSmoother (`path_smoother.cpp`)
- **Cubic Spline**: Natural cubic spline via Thomas algorithm (tridiagonal solver)
- **Gradient Descent**: Iterative smoothing minimizing `w_data * ‖path - original‖² + w_smooth * ‖Δ²path‖²`
- Input: raw waypoints `(x, y)`
- Output: densely sampled `Pose2D` with computed headings

### TrajectoryGenerator (`trajectory_generator.cpp`)
- Arc-length parameterization of smoothed path
- **Trapezoidal velocity profile**: acceleration → cruise → deceleration
- Handles triangular profile when path is too short for full cruise phase
- Computes curvature via finite differences
- Output: timestamped `TrajectoryPoint` with `(x, y, θ, v, ω, κ, t, s)`

### TrajectoryController (`trajectory_controller.cpp`)
- **Pure Pursuit**: Geometric tracker finding look-ahead point on trajectory
  - Curvature command: `κ = 2 * y_local / L_d²`
- **PID cross-track correction**: Reduces speed proportional to lateral error
- Nearest-point search with sliding window for efficiency
- Goal tolerance check for trajectory completion

---

## File Structure

```
src/trajectory_control/
├── CMakeLists.txt
├── package.xml
├── include/trajectory_control/
│   ├── types.hpp                    # Pose2D, TrajectoryPoint, Obstacle
│   ├── path_smoother.hpp
│   ├── trajectory_generator.hpp
│   └── trajectory_controller.hpp
├── src/
│   ├── path_smoother.cpp
│   ├── trajectory_generator.cpp
│   ├── trajectory_controller.cpp
│   └── trajectory_node.cpp          # Main ROS2 node
├── launch/
│   ├── simulation.launch.py         # Gazebo + TurtleBot3
│   └── trajectory_demo.launch.py    # Full demo
├── config/
│   ├── controller_params.yaml
│   ├── waypoints.yaml
│   └── trajectory.rviz
├── test/
│   ├── test_path_smoother.cpp
│   ├── test_trajectory_generator.cpp
│   └── test_controller.cpp
├── scripts/
│   ├── plot_trajectory.py
│   └── waypoint_publisher.py
└── worlds/
    └── obstacle_world.sdf
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `smoothing_method` | `cubic_spline` | `cubic_spline` or `gradient_descent` |
| `num_smooth_points` | 200 | Interpolation density |
| `max_velocity` | 0.18 m/s | TurtleBot3 Burger limit |
| `max_acceleration` | 0.5 m/s² | Trapezoidal ramp rate |
| `velocity_profile` | `trapezoidal` | `trapezoidal` or `constant` |
| `look_ahead_distance` | 0.3 m | Pure Pursuit L_d |
| `goal_tolerance` | 0.08 m | Completion threshold |
| `pid_kp / ki / kd` | 1.0 / 0.0 / 0.1 | Cross-track PID gains |

---

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/odom` | nav_msgs/Odometry | Subscribe |
| `/cmd_vel` | geometry_msgs/Twist | Publish |
| `/smoothed_path` | nav_msgs/Path | Publish |
| `/actual_path` | nav_msgs/Path | Publish |
| `/original_waypoints` | visualization_msgs/MarkerArray | Publish |
| `/tracking_error` | std_msgs/Float64 | Publish |

---

## License

Academic assignment — University use only.
