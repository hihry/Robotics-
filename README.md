# smooth_nav — TurtleBot3 Path Smoothing & Trajectory Control

Professional 8-package ROS 2 Humble architecture for **2D path smoothing**, **trajectory generation**, and **trajectory tracking** on TurtleBot3 Burger in Gazebo Classic simulation.

## Assignment Overview

| Component | Points | Algorithm |
|-----------|--------|-----------|
| Path Smoothing | 25 | Cubic Spline (Thomas algorithm) + B-Spline Gradient Descent |
| Trajectory Generation | 25 | Trapezoidal Velocity Profile (arc-length parameterized) |
| Trajectory Tracking | 25 | Pure Pursuit + PID Heading Correction (Action Server) |
| Code Quality & Testing | 15 | 30+ GTest cases, Strategy/Factory patterns, zero-ROS core |
| Documentation & Demo | 10 | Algorithms doc, design decisions, extension guides |

---

## Architecture

```
                         ┌──────────────────────┐
                         │  smooth_nav_bringup   │  ← Master launch (orchestrator)
                         └──────────┬───────────┘
            ┌───────────────────────┼───────────────────────┐
            ▼                       ▼                       ▼
   smooth_nav_simulation   smooth_nav_ros          smooth_nav_controller
   (Gazebo + worlds)       (Service nodes)         (Action server)
            │               ┌───┴───┐                      │
            │               ▼       ▼                      ▼
            │          Smoother  Generator         Trajectory Tracker
            │          Service   Service           /execute_trajectory
            │               │       │                      │
            │               └───┬───┘                      │
            │                   ▼                          │
            │            smooth_nav_core  ◄────────────────┘
            │            (Pure C++17, zero ROS)
            │                   │
            ▼                   ▼
   smooth_nav_description  smooth_nav_msgs
   (URDF, RViz config)    (Custom messages)
```

### Pipeline Flow

```
Waypoints → [SmoothPath Service] → SmoothedPath → [GenerateTrajectory Service] → Trajectory
                                                                                      ↓
                      /cmd_vel ← [ExecuteTrajectory Action] ← /odom + Trajectory
```

---

## Quick Start (Docker)

### Prerequisites
- **Docker** ≥ 20.x (Docker Desktop on Windows/macOS)
- **VcXsrv** or **XQuartz** (X Server for GUI — Gazebo/RViz2)

### 1. Build & enter development container
```bash
cd docker
docker compose build dev
docker compose run --rm dev
```

### 2. Build workspace (inside container)
```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch full demo
```bash
# Start VcXsrv first (Windows) with "Disable access control" checked
ros2 launch smooth_nav_bringup smooth_nav.launch.py
```

### 4. Run unit tests
```bash
colcon test --packages-select smooth_nav_core --return-code-on-test-failure
colcon test-result --verbose
```

### 5. Run all tests (unit + integration + system)
```bash
./scripts/run_tests.sh
```

---

## Package Overview

| Package | Purpose |
|---------|---------|
| `smooth_nav_msgs` | Custom .msg, .srv, .action definitions |
| `smooth_nav_core` | Pure C++17 algorithms — zero ROS deps, GTest tested |
| `smooth_nav_ros` | ROS 2 service nodes (SmoothPath, GenerateTrajectory) |
| `smooth_nav_controller` | ROS 2 action server (ExecuteTrajectory) |
| `smooth_nav_description` | TurtleBot3 URDF, RViz config |
| `smooth_nav_simulation` | Gazebo worlds + launch |
| `smooth_nav_bringup` | Master launch files, waypoint configs |
| `smooth_nav_tests` | Integration & system-level tests |

---

## File Structure

```
RoboticsAssignment/
├── .github/workflows/          # CI/CD (build, test, lint)
├── docker/
│   ├── Dockerfile.dev          # Full development image
│   ├── Dockerfile.ci           # Headless CI image
│   ├── docker-compose.yml      # dev, test, sim services
│   └── entrypoint.sh
├── docs/
│   ├── algorithms.md           # Mathematical details
│   ├── design_decisions.md     # Architecture rationale
│   ├── real_robot_extension.md # Physical TurtleBot3 guide
│   └── obstacle_avoidance_extension.md
├── scripts/
│   ├── setup_workspace.sh      # Build + test helper
│   ├── run_tests.sh            # All-tests runner
│   ├── visualize_trajectory.py # Matplotlib plots
│   └── record_bag.sh           # ROS 2 bag recorder
├── src/
│   ├── smooth_nav_msgs/        # Messages, services, actions
│   ├── smooth_nav_core/        # Pure C++17 algorithms + GTests
│   ├── smooth_nav_ros/         # Service wrapper nodes
│   ├── smooth_nav_controller/  # Action server node
│   ├── smooth_nav_description/ # URDF, RViz, meshes
│   ├── smooth_nav_simulation/  # Gazebo worlds + launch
│   ├── smooth_nav_bringup/     # Master launch + configs
│   └── smooth_nav_tests/       # Integration & system tests
├── .clang-format
├── .pre-commit-config.yaml
├── CHANGELOG.md
└── README.md
```

---

## Design Patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **Strategy** | `IPathSmoother`, `ITrajectoryGenerator`, `IController` | Swap algorithms via YAML config |
| **Factory** | `SmootherFactory::create(type)` | Instantiate strategies by name |
| **Interface Segregation** | Separate abstract interfaces | Each node depends only on what it uses |
| **Config Over Code** | YAML params loaded at launch | Tune without recompiling |

---

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `smoother_type` | `cubic_spline` | `cubic_spline` or `bspline` |
| `num_smooth_points` | 200 | Interpolation density |
| `generator_type` | `trapezoidal` | `trapezoidal` or `constant` |
| `max_velocity` | 0.22 m/s | TurtleBot3 Burger max |
| `max_acceleration` | 0.5 m/s² | Trapezoidal ramp rate |
| `look_ahead_distance` | 0.3 m | Pure Pursuit L_d |
| `goal_tolerance` | 0.05 m | Completion threshold |
| `kp / ki / kd` | 2.0 / 0.0 / 0.1 | Heading PID gains |
| `control_rate` | 20.0 Hz | Controller loop frequency |

---

## ROS 2 Interfaces

### Services
| Service | Type | Node |
|---------|------|------|
| `~/smooth_path` | `smooth_nav_msgs/SmoothPath` | path_smoother_node |
| `~/generate_trajectory` | `smooth_nav_msgs/GenerateTrajectory` | trajectory_generator_node |

### Actions
| Action | Type | Node |
|--------|------|------|
| `~/execute_trajectory` | `smooth_nav_msgs/ExecuteTrajectory` | trajectory_tracker_node |

### Topics (Published)
| Topic | Type | Node |
|-------|------|------|
| `/smoothed_path` | `nav_msgs/Path` | path_smoother_node |
| `/original_waypoints` | `visualization_msgs/MarkerArray` | path_smoother_node |
| `/cmd_vel` | `geometry_msgs/Twist` | trajectory_tracker_node |
| `/actual_path` | `nav_msgs/Path` | trajectory_tracker_node |
| `/tracking_error` | `std_msgs/Float64` | trajectory_tracker_node |
| `/controller_diagnostics` | `smooth_nav_msgs/ControllerDiagnostics` | trajectory_tracker_node |

---

## Testing

- **30+ unit tests** in `smooth_nav_core` (cubic spline, velocity profiles, pure pursuit, PID, geometry)
- **Integration tests** in `smooth_nav_tests` (service calls, action execution)
- **System launch test** (verifies all nodes start and advertise interfaces)

```bash
# Unit tests only
colcon test --packages-select smooth_nav_core

# All tests
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

---

## Documentation

- [Algorithms](docs/algorithms.md) — Mathematical derivations and formulations
- [Design Decisions](docs/design_decisions.md) — Architecture rationale
- [Real Robot Extension](docs/real_robot_extension.md) — Sim-to-real guide
- [Obstacle Avoidance Extension](docs/obstacle_avoidance_extension.md) — Adding reactive avoidance

---

## License

Academic assignment — University use only.
