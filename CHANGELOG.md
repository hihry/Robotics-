# Changelog

All notable changes to this project will be documented in this file.

## [2.0.0] — 2026-02-26

### Added
- **Safety watchdog node** (`safety_watchdog_node.py`) — `/cmd_vel_raw` → `/cmd_vel` interposer with velocity clamping, acceleration limiting, command timeout, and laser proximity stop
- **Pipeline orchestrator** (`waypoint_client_node.py`) — end-to-end Smooth → Generate → Execute with retry, abort, and RViz markers
- **Curvature-based speed limiting** in trajectory generator ($v \leq \sqrt{a_{lat,max}/|\kappa|}$)
- **Adaptive look-ahead distance** in trajectory tracker ($L_d = L_{d,base} + k \cdot |v|$)
- **Goal deceleration** — smooth deceleration within configurable radius of final waypoint
- **Dynamic parameter reconfiguration** — all C++ nodes support `rqt_reconfigure` live tuning
- **Rich RViz visualization** — curvature markers, velocity profile, trajectory path, command arrows, waypoint spheres, safety status text overlay
- `/pipeline_status` topic with coordinated state machine feedback
- Signed cross-track error via cross product for PID correction

### Changed
- Upgraded `path_smoother_node` — publishes curvature color-mapped markers, dynamic reconfig
- Upgraded `trajectory_generator_node` — curvature-limited profiles, publishes velocity profile markers
- Upgraded `trajectory_tracker_node` — adaptive look-ahead, goal deceleration, publishes velocity command arrows
- Unit tests expanded from 30 → **64 GTest cases + 1 lint test** (all passing)
- Integration tests fixed: msg field mismatches (`smooth_path` → `smoothed_path`, `points` → `waypoints`, `total_time` → `duration`)
- `demo.launch.py` launches full pipeline including safety watchdog
- Updated documentation: algorithms, design decisions, README

### Fixed
- Trajectory generator trapezoidal profile handles zero-length segments
- Pure Pursuit handles edge case when closest point is the last waypoint

---

## [1.0.0] — 2025-12-01

### Added
- 8-package `smooth_nav` architecture
- Natural cubic spline path smoothing
- B-spline gradient descent smoothing
- Trapezoidal and constant velocity trajectory generation
- Pure Pursuit + PID trajectory tracking controller
- ROS 2 service nodes for smoothing and generation
- ROS 2 action server for trajectory execution
- TurtleBot3 Burger URDF and RViz configuration
- Gazebo Classic worlds (empty + obstacles)
- Docker development environment
- GitHub Actions CI/CD
- Comprehensive unit tests (30+ test cases)
- Integration and system-level tests
- Documentation: algorithms, design decisions, extensions
