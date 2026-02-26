# Design Decisions — smooth_nav

This document explains the key architectural and design decisions made in this project.

---

## 1. Multi-Package Architecture

**Decision:** Split the project into 8 separate ROS 2 packages instead of a monolithic package.

**Rationale:**
- **Separation of concerns** — each package has a single responsibility
- **Independent testing** — `smooth_nav_core` can be tested without ROS 2
- **Reusability** — core algorithms can be used in non-ROS contexts
- **Faster incremental builds** — changing a launch file doesn't recompile C++

---

## 2. Zero-ROS Core Library

**Decision:** `smooth_nav_core` has NO ROS 2 dependencies. Pure C++17 only.

**Rationale:**
- Unit tests run with plain GTest, no `rclcpp` spin required
- Algorithms are portable to other frameworks (ROS 1, standalone apps)
- Enforced by CMakeLists.txt: only `find_package(GTest)`, no `find_package(rclcpp)`

---

## 3. Strategy + Factory Patterns

**Decision:** All algorithm families (smoothing, trajectory generation, control) use the Strategy pattern with a Factory for instantiation.

**Rationale:**
- Adding a new smoother = one `.hpp` + one `.cpp` + one factory registration
- Runtime algorithm selection via YAML parameter (no recompile)
- Easy A/B comparison between algorithms

---

## 4. Action Server for Trajectory Execution

**Decision:** Use a ROS 2 Action (not a simple service) for trajectory tracking.

**Rationale:**
- Trajectory execution is long-running — Actions support feedback and cancellation
- Progress percentage, cross-track error, and heading error reported as feedback
- Graceful cancellation: sends zero velocity on cancel

---

## 5. Service Nodes for Smoothing & Generation

**Decision:** Path smoothing and trajectory generation are exposed as ROS 2 Services.

**Rationale:**
- These are request/response operations (not continuous)
- Services ensure the caller waits for the result before proceeding
- Matches the pipeline: smooth → generate → execute

---

## 6. Thread Safety

**Decision:** Odometry updates in the tracker node are protected by `std::mutex`.

**Rationale:**
- The `/odom` subscriber callback and the control-loop timer run on different executor threads
- Mutex ensures consistent pose reads during control computation

---

## 7. Docker-First Development

**Decision:** All development and testing happens inside Docker containers.

**Rationale:**
- Reproducible across any host OS (Windows, macOS, Linux)
- No "works on my machine" issues
- CI uses the same Docker image
- TurtleBot3 + Gazebo dependencies are managed in the Dockerfile

---

## 8. Safety Watchdog as Separate Node

**Decision:** A dedicated `safety_watchdog_node` sits between the controller (`/cmd_vel_raw`) and the motor driver (`/cmd_vel`), rather than embedding safety logic inside the controller.

**Rationale:**
- **Single Responsibility** — the controller focuses on trajectory tracking; the watchdog focuses on hardware protection
- **Re-usable** — the same watchdog works with any controller (Pure Pursuit, nav2, teleop)
- **Fail-safe** — if the controller node crashes, the watchdog's timeout triggers an emergency stop
- **Transparent** — services and action server don't need to know about safety limits

---

## 9. Pipeline Orchestrator Node

**Decision:** A Python `waypoint_client_node` sequences the entire Smooth → Generate → Execute pipeline, rather than requiring the user to call each service manually.

**Rationale:**
- **End-to-end demo** — `ros2 launch smooth_nav_bringup demo.launch.py` runs everything hands-free
- **Retry with exponential backoff** — handles service startup delays gracefully
- **Abort mechanism** — publish to `/abort_mission` to stop at any point
- **Visualization** — publishes waypoint and trajectory markers for RViz

---

## 10. Dynamic Parameter Reconfiguration

**Decision:** All C++ nodes use `add_on_set_parameters_callback` for live tuning, with input validation.

**Rationale:**
- Tune PID gains, look-ahead distance, velocity limits via `rqt_reconfigure` while the robot is running
- No restart = faster iteration in simulation and on real hardware
- Validation callbacks reject invalid values (e.g., negative velocities) before they cause crashes

---

## 11. Curvature-Based Speed Limiting

**Decision:** The trajectory generator enforces $v \leq \sqrt{a_{lat,max} / |\kappa|}$ at every path point.

**Rationale:**
- Without this, the robot attempts sharp turns at full speed → wheel slip, tracking divergence
- `max_lateral_acceleration` is exposed as a ROS parameter so it can be tuned per environment
- This matches how industrial AGVs and autonomous cars handle curvature-limited corridors

---

## 12. Adaptive Look-Ahead Distance

**Decision:** $L_d = L_{d,base} + k \cdot |v|$ instead of a fixed look-ahead.

**Rationale:**
- At low speed: small $L_d$ → tight tracking through curves
- At high speed: large $L_d$ → smooth, stable pursuit without oscillation
- Classic Pure Pursuit uses fixed $L_d$ and requires careful per-path tuning; adaptive scales automatically
