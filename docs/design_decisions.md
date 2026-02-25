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
