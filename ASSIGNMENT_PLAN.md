# Robotics Assignment: Path Smoothing & Trajectory Control in 2D Space
## Complete 2-Day Execution Plan (Feb 25-26, 2026)

---

## ASSIGNMENT SUMMARY

| Component | Points | Weight |
|---|---|---|
| Code Quality (Trajectory Gen & Smoothing 15 + Controller 20) | 35 | 35% |
| Code Architecture & Comments | 10 | 10% |
| Simulation | 20 | 20% |
| Testability & QA (Test Case Design 10, Automation 5, Error Handling 5) | 20 | 20% |
| Documentation, Reports & Video | 15 | 15% |
| **Total** | **100** | |
| **Extra Credit**: Obstacle Avoidance Extension | Bonus | |

---

## TECHNOLOGY STACK DECISION

| Layer | Choice | Reason |
|---|---|---|
| **Language** | C++ (primary) + Python (prototyping/plotting) | Assignment explicitly mentions C++ with ROS2 |
| **Framework** | ROS2 Humble | Most stable LTS, best TurtleBot3 support |
| **Simulator** | Gazebo (Classic or Ignition) | Tightly integrated with ROS2 & TurtleBot3 |
| **Robot Platform** | TurtleBot3 Burger (differential drive) | Recommended in assignment, simplest model |
| **Path Smoothing** | Cubic Spline Interpolation | Smooth, C2 continuous, well-understood |
| **Trajectory Generation** | Trapezoidal Velocity Profile | Industry standard, easy to implement |
| **Controller** | Pure Pursuit + PID | Well-suited for differential drive |
| **Build System** | colcon + CMake | ROS2 standard |
| **Testing** | Google Test (C++) + pytest (Python) + launch_testing | ROS2 native testing |
| **Visualization** | RViz2 + Matplotlib (for plots) | Native ROS2 viz + publication-quality plots |

---

## AI TOOLS & RESOURCES TO USE

### AI Assistants (Use Throughout)
| Tool | Purpose | URL |
|---|---|---|
| **GitHub Copilot (me!)** | Code generation, debugging, architecture advice | Already active |
| **Claude (Anthropic)** | Complex algorithm explanations, mathematical derivations | https://claude.ai |
| **ChatGPT/GPT-4** | Alternative perspective, ROS2-specific questions | https://chat.openai.com |
| **Perplexity AI** | Quick research with citations (papers, docs) | https://perplexity.ai |

### Code & Algorithm References
| Resource | Purpose | URL |
|---|---|---|
| **PythonRobotics** | Reference implementations for ALL algorithms | https://github.com/AtsushiSakai/PythonRobotics |
| **Nav2 Documentation** | ROS2 navigation architecture patterns | https://docs.nav2.org/ |
| **TurtleBot3 e-Manual** | Simulation setup, robot specs | https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/ |
| **ROS2 Humble Docs** | API references, tutorials | https://docs.ros.org/en/humble/ |

### Visualization & Documentation Tools
| Tool | Purpose | URL |
|---|---|---|
| **PlotJuggler** | Real-time ROS2 topic plotting | `sudo apt install ros-humble-plotjuggler-ros` |
| **Foxglove Studio** | Advanced robotics visualization | https://foxglove.dev/ |
| **Draw.io / Excalidraw** | Architecture diagrams | https://app.diagrams.net / https://excalidraw.com |
| **Mermaid** | Code-based diagrams in README | Built into GitHub Markdown |
| **OBS Studio** | Screen recording for demo video | https://obsproject.com/ |
| **Manim / Matplotlib** | Publication-quality trajectory plots | pip install manim / matplotlib |

### Testing & Quality Tools
| Tool | Purpose |
|---|---|
| **Google Test** | C++ unit testing framework |
| **launch_testing** | ROS2 integration testing |
| **clang-format** | C++ code formatting |
| **cppcheck / clang-tidy** | Static analysis |
| **rosbag2** | Recording simulation data for replay tests |
| **colcon test** | Unified test runner |

---

## MATHEMATICAL BACKGROUND (Quick Reference)

### 1. Cubic Spline Interpolation
Given waypoints $(x_i, y_i)$, fit piecewise cubic polynomials:
$$S_i(t) = a_i + b_i(t-t_i) + c_i(t-t_i)^2 + d_i(t-t_i)^3$$
- Guarantees C2 continuity (position, velocity, acceleration continuous)
- Use `scipy.interpolate.CubicSpline` for prototyping, then port to C++ using Eigen

### 2. Trapezoidal Velocity Profile
Three phases: acceleration, cruise, deceleration
$$v(t) = \begin{cases} a_{max} \cdot t & \text{acceleration phase} \\ v_{max} & \text{cruise phase} \\ v_{max} - a_{max}(t - t_2) & \text{deceleration phase} \end{cases}$$

### 3. Pure Pursuit Controller
Look-ahead point on trajectory, compute curvature:
$$\kappa = \frac{2 \cdot \sin(\alpha)}{L_d}$$
where $\alpha$ = angle to look-ahead point, $L_d$ = look-ahead distance

### 4. Differential Drive Kinematics
$$\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega$$
Convert $(v, \omega)$ to wheel velocities: $v_L = v - \omega \cdot d/2$, $v_R = v + \omega \cdot d/2$

---

## HOUR-BY-HOUR PLAN

### ═══════════════════════════════════════
### DAY 1 (Feb 25) — Foundation + Core Algorithms
### ═══════════════════════════════════════

---

### HOUR 1 (0:00–1:00) — Environment Setup & Project Scaffolding
**Priority: CRITICAL**

**Tasks:**
- [ ] Install/verify ROS2 Humble on Ubuntu (WSL2 or dual boot)
  ```bash
  # If not installed:
  sudo apt update && sudo apt install ros-humble-desktop
  sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
  pip install transforms3d numpy scipy matplotlib
  ```
- [ ] Set up TurtleBot3 simulation packages
  ```bash
  export TURTLEBOT3_MODEL=burger
  mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
  git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  cd ~/turtlebot3_ws && colcon build --symlink-install
  ```
- [ ] Verify Gazebo + TurtleBot3 launches successfully
  ```bash
  source ~/turtlebot3_ws/install/setup.bash
  ros2 launch turtlebot3_gazebo empty_world.launch.py
  ```
- [ ] Create the ROS2 workspace & package structure:
  ```
  robotics_assignment_ws/
  ├── src/
  │   └── trajectory_control/
  │       ├── CMakeLists.txt
  │       ├── package.xml
  │       ├── include/trajectory_control/
  │       │   ├── path_smoother.hpp
  │       │   ├── trajectory_generator.hpp
  │       │   ├── trajectory_controller.hpp
  │       │   └── utils.hpp
  │       ├── src/
  │       │   ├── path_smoother.cpp
  │       │   ├── trajectory_generator.cpp
  │       │   ├── trajectory_controller.cpp
  │       │   ├── trajectory_node.cpp (main ROS2 node)
  │       │   └── utils.cpp
  │       ├── launch/
  │       │   ├── simulation.launch.py
  │       │   └── trajectory_demo.launch.py
  │       ├── config/
  │       │   ├── waypoints.yaml
  │       │   └── controller_params.yaml
  │       ├── test/
  │       │   ├── test_path_smoother.cpp
  │       │   ├── test_trajectory_generator.cpp
  │       │   ├── test_controller.cpp
  │       │   └── test_integration.py
  │       ├── scripts/
  │       │   ├── plot_trajectory.py
  │       │   └── waypoint_publisher.py
  │       └── worlds/
  │           └── obstacle_world.sdf
  ```
- [ ] Initialize Git repo, create `.gitignore`

**AI Tool Usage:** Ask Copilot to generate the CMakeLists.txt and package.xml boilerplate

**Deliverable:** Working ROS2 workspace that compiles, TurtleBot3 spawns in Gazebo

---

### HOUR 2 (1:00–2:00) — Path Smoothing Algorithm (Core Math)
**Priority: HIGH (15 pts)**

**Tasks:**
- [ ] Implement `PathSmoother` class in C++
  - Constructor takes vector of waypoints `std::vector<std::pair<double, double>>`
  - Method: `smoothPath()` returns densely sampled smooth path
  - Algorithm: **Cubic Spline Interpolation**
    - Parameterize by cumulative arc length
    - Separate splines for x(s) and y(s)
    - Use Eigen library for solving tridiagonal system
  - Alternative (simpler): **Gradient Descent Smoothing**
    ```
    for each iteration:
      for each non-endpoint waypoint i:
        new_path[i] += alpha * (original[i] - new_path[i])  // data weight
        new_path[i] += beta * (new_path[i-1] + new_path[i+1] - 2*new_path[i])  // smooth weight
    ```
- [ ] Add configurable parameters: smoothing weight, number of interpolation points
- [ ] Implement at least 2 smoothing methods for comparison (cubic spline + gradient descent)

**Reference Code:** `PythonRobotics/PathPlanning/CubicSpline/cubic_spline_planner.py`

**AI Tool Usage:**
- Ask Copilot to generate Eigen-based tridiagonal solver
- Use Claude to verify the mathematical derivation of natural cubic spline boundary conditions
- Reference: https://en.wikipedia.org/wiki/Spline_interpolation

**Deliverable:** `path_smoother.hpp/.cpp` with clean API and 2 algorithm options

---

### HOUR 3 (2:00–3:00) — Path Smoothing (ROS2 Integration + Testing)
**Priority: HIGH**

**Tasks:**
- [ ] Create a ROS2 node that:
  - Subscribes to waypoints (or loads from YAML config)
  - Publishes smoothed path as `nav_msgs/msg/Path`
  - Publishes original waypoints as `visualization_msgs/msg/MarkerArray`
- [ ] Visualize in RViz2: original waypoints (red markers) vs smoothed path (green line)
- [ ] Write unit tests for `PathSmoother`:
  - Test: straight line input → straight line output
  - Test: L-shaped path → smooth curve
  - Test: single point → handled gracefully
  - Test: two points → linear interpolation
  - Test: empty input → appropriate error
- [ ] Test with sample waypoints:
  ```yaml
  waypoints:
    - [0.0, 0.0]
    - [1.0, 0.0]
    - [2.0, 1.0]
    - [3.0, 1.0]
    - [4.0, 0.0]
  ```

**AI Tool Usage:** Ask Copilot to generate Google Test boilerplate for edge cases

**Deliverable:** Smoothed path visible in RViz2, passing unit tests

---

### HOUR 4 (3:00–4:00) — Trajectory Generation (Time Parameterization)
**Priority: HIGH (Part of 15 pts)**

**Tasks:**
- [ ] Implement `TrajectoryGenerator` class:
  - Input: smooth path `std::vector<Pose2D>`
  - Output: time-stamped trajectory `std::vector<TrajectoryPoint>` where:
    ```cpp
    struct TrajectoryPoint {
        double x, y, theta;  // pose
        double v, omega;     // velocities
        double timestamp;    // time
    };
    ```
  - **Trapezoidal Velocity Profile:**
    - Parameters: `v_max`, `a_max`, `v_start=0`, `v_end=0`
    - Compute total arc length
    - Divide into accel/cruise/decel phases
    - Map time to arc length to (x,y) position
  - **Constant Velocity Profile** (simpler alternative):
    - Uniform time spacing based on desired speed
  - Compute heading (theta) from path tangent: `theta = atan2(dy, dx)`
  - Compute curvature for each point: `κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)`
- [ ] Publish trajectory as custom message or `nav_msgs/msg/Path` with timestamps

**AI Tool Usage:**
- Use Copilot to implement trapezoidal velocity profile math
- Use Perplexity AI to search for "trapezoidal velocity profile robotics implementation"

**Deliverable:** `trajectory_generator.hpp/.cpp` producing time-parameterized trajectory

---

### HOUR 5 (4:00–5:00) — Trajectory Generation (Validation + Visualization)
**Priority: HIGH**

**Tasks:**
- [ ] Create Python plotting script (`scripts/plot_trajectory.py`):
  - Plot 1: 2D path (x vs y) — original waypoints, smoothed path, trajectory samples
  - Plot 2: Velocity profile (v vs t) — showing trapezoidal shape
  - Plot 3: Angular velocity profile (ω vs t)
  - Plot 4: Curvature profile (κ vs s)
  - Plot 5: x(t) and y(t) separately
- [ ] Write unit tests for `TrajectoryGenerator`:
  - Test: straight line → zero angular velocity
  - Test: velocity profile respects v_max and a_max
  - Test: timestamps are monotonically increasing
  - Test: total time is physically reasonable
  - Test: trajectory starts and ends at correct positions

**AI Tool Usage:** Ask Copilot to generate matplotlib visualization code

**Deliverable:** Beautiful plots showing trajectory profiles, passing tests

---

### HOUR 6 (5:00–6:00) — Trajectory Tracking Controller (Core)
**Priority: CRITICAL (20 pts)**

**Tasks:**
- [ ] Implement `TrajectoryController` class with **Pure Pursuit** algorithm:
  ```cpp
  class TrajectoryController {
  public:
      // Returns (v, omega) command
      std::pair<double, double> computeControl(
          const Pose2D& current_pose,
          const std::vector<TrajectoryPoint>& trajectory,
          double current_time
      );
  private:
      double look_ahead_distance_;
      double max_linear_vel_;
      double max_angular_vel_;
      // Find the trajectory point ahead of current position
      TrajectoryPoint findLookAheadPoint(...);
  };
  ```
- [ ] Pure Pursuit Logic:
  1. Find nearest point on trajectory to robot
  2. Find look-ahead point at distance `L_d` ahead
  3. Compute curvature: `κ = 2*sin(α) / L_d`
  4. Compute angular velocity: `ω = v * κ`
  5. Clamp outputs to robot limits
- [ ] Add PID controller for velocity tracking as secondary layer
- [ ] Add error metrics computation (cross-track error, heading error)

**Reference:**
- `PythonRobotics/PathTracking/pure_pursuit/pure_pursuit.py`
- Paper: "Implementation of the Pure Pursuit Path Tracking Algorithm" (CMU)

**AI Tool Usage:**
- Ask Copilot for pure pursuit implementation adapted for differential drive
- Use Claude for debugging edge cases (e.g., robot overshooting waypoints)

**Deliverable:** `trajectory_controller.hpp/.cpp` with clean Pure Pursuit + PID implementation

---

### HOUR 7 (6:00–7:00) — Controller ROS2 Integration
**Priority: CRITICAL**

**Tasks:**
- [ ] Create main ROS2 node `trajectory_node.cpp`:
  ```
  Subscribers:
    - /odom (nav_msgs/Odometry) — robot's current pose
  Publishers:
    - /cmd_vel (geometry_msgs/Twist) — velocity commands
    - /smoothed_path (nav_msgs/Path) — visualization
    - /tracking_error (std_msgs/Float64) — cross-track error
  Parameters:
    - look_ahead_distance, max_vel, max_omega, waypoints_file
  ```
- [ ] Implement the control loop:
  1. Load waypoints → Smooth path → Generate trajectory
  2. At each /odom callback: compute control, publish /cmd_vel
  3. Log tracking errors
- [ ] Create `launch/trajectory_demo.launch.py`:
  - Launch Gazebo with TurtleBot3
  - Launch trajectory controller node
  - Launch RViz2 with saved config

**AI Tool Usage:** Ask Copilot for ROS2 node boilerplate with lifecycle management

**Deliverable:** Robot moves in Gazebo following basic trajectory

---

### HOUR 8 (7:00–8:00) — Initial Simulation Testing & Debugging
**Priority: HIGH**

**Tasks:**
- [ ] Run full simulation pipeline end-to-end
- [ ] Debug common issues:
  - TF frame mismatches (odom → base_link)
  - Coordinate frame conventions
  - Controller oscillation / overshoot
  - Robot not moving (check /cmd_vel topic)
- [ ] Tune controller parameters:
  - Look-ahead distance: start with 0.3m, adjust
  - Linear velocity: 0.15 m/s (TurtleBot3 Burger max ~0.22 m/s)
  - Angular velocity: clamp to ±2.84 rad/s
- [ ] Record a rosbag for replay testing:
  ```bash
  ros2 bag record /odom /cmd_vel /smoothed_path -o test_run_1
  ```

**AI Tool Usage:** Use PlotJuggler to visualize real-time tracking performance

**Deliverable:** Robot tracks a simple trajectory (e.g., square path) in Gazebo

---

### HOUR 9 (8:00–9:00) — Controller Tuning & Performance Optimization
**Priority: HIGH**

**Tasks:**
- [ ] Tune PID + Pure Pursuit parameters systematically:
  - Create a parameter sweep script
  - Test with different waypoint configurations:
    - Straight line
    - 90-degree turns 
    - S-curve
    - Circle
    - Figure-8
- [ ] Reduce cross-track error to < 0.05m
- [ ] Ensure smooth velocity transitions (no jerky motion)
- [ ] Add adaptive look-ahead distance (increase at higher speeds)
- [ ] Test with different velocity profiles (constant vs trapezoidal)

**AI Tool Usage:**
- Ask Copilot for parameter tuning scripts
- Use Claude to understand stability analysis of Pure Pursuit

**Deliverable:** Well-tuned controller with < 5cm tracking error on test paths

---

### HOUR 10 (9:00–10:00) — Error Handling & Robustness
**Priority: MEDIUM (5 pts Error Handling)**

**Tasks:**
- [ ] Add comprehensive error handling:
  - Invalid waypoints (NaN, Inf, duplicates, too close together)
  - Empty trajectory
  - Robot far from trajectory (re-planning trigger)
  - Velocity limits exceeded
  - Timeout / trajectory completion detection
- [ ] Add trajectory completion detection (within tolerance of final point)
- [ ] Add safety features:
  - Emergency stop if error exceeds threshold
  - Watchdog timer for /odom updates
  - Graceful shutdown
- [ ] Add ROS2 parameter validation with descriptors
- [ ] Add meaningful log messages (RCLCPP_INFO, WARN, ERROR)

**AI Tool Usage:** Ask Copilot to generate error handling patterns for each edge case

**Deliverable:** Robust system that handles edge cases gracefully

---

### HOUR 11-12 (10:00–12:00) — Day 1 Review, Code Cleanup & Rest
**Priority: MEDIUM**

**Tasks:**
- [ ] Review all code written today
- [ ] Add/improve inline comments and docstrings
- [ ] Ensure consistent code formatting (run `clang-format`)
- [ ] Commit all changes with meaningful messages
- [ ] Make notes on what needs work tomorrow
- [ ] **REST — You need sleep for a productive Day 2!**

**Git commits should look like:**
```
feat: implement cubic spline path smoothing with gradient descent alternative
feat: add trapezoidal velocity profile trajectory generation
feat: implement pure pursuit controller for differential drive
feat: create main ROS2 node with full pipeline integration
test: add unit tests for path smoother and trajectory generator
fix: resolve TF frame mismatch in controller node
```

---

### ═══════════════════════════════════════
### DAY 2 (Feb 26) — Testing, Documentation, Polish & Video
### ═══════════════════════════════════════

---

### HOUR 13 (0:00–1:00) — Test Suite Development
**Priority: HIGH (10 pts Test Case Design)**

**Tasks:**
- [ ] Write comprehensive Google Test suite for C++ code:
  ```cpp
  // test_path_smoother.cpp
  TEST(PathSmootherTest, StraightLineRemainsLinear) {...}
  TEST(PathSmootherTest, SmoothedPathPassesThroughEndpoints) {...}
  TEST(PathSmootherTest, OutputDenserThanInput) {...}
  TEST(PathSmootherTest, C2Continuity) {...}  // Check derivatives
  TEST(PathSmootherTest, EmptyInputHandled) {...}
  TEST(PathSmootherTest, SinglePointHandled) {...}
  TEST(PathSmootherTest, TwoPointsLinearInterpolation) {...}
  
  // test_trajectory_generator.cpp
  TEST(TrajectoryGenTest, TimestampsMonotonicallyIncreasing) {...}
  TEST(TrajectoryGenTest, VelocityProfileRespectsLimits) {...}
  TEST(TrajectoryGenTest, TrapezoidalProfileShape) {...}
  TEST(TrajectoryGenTest, ZeroLengthPath) {...}
  TEST(TrajectoryGenTest, StartEndVelocitiesCorrect) {...}
  
  // test_controller.cpp  
  TEST(ControllerTest, ZeroErrorZeroCommand) {...}
  TEST(ControllerTest, OutputWithinVelocityLimits) {...}
  TEST(ControllerTest, ConvergesToTrajectory) {...}
  TEST(ControllerTest, HandlesTrajectoryCompletion) {...}
  ```
- [ ] Add edge case tests (NaN inputs, extreme values, boundary conditions)

**AI Tool Usage:** Ask Copilot to generate test cases systematically

**Deliverable:** 15+ unit tests covering all components

---

### HOUR 14 (1:00–2:00) — Test Automation & Integration Tests
**Priority: HIGH (5 pts Test Automation)**

**Tasks:**
- [ ] Set up CMakeLists.txt for automated testing:
  ```cmake
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_smoother test/test_path_smoother.cpp)
  target_link_libraries(test_path_smoother path_smoother_lib)
  
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_integration.py)
  ```
- [ ] Create integration test with `launch_testing`:
  ```python
  # test/test_integration.py
  # Launches full stack, sends waypoints, verifies robot reaches goal
  ```
- [ ] Ensure `colcon test` runs all tests and passes
- [ ] Add test coverage metrics if time allows (`gcov`/`lcov`)
- [ ] Create a test waypoint configuration file for reproducible testing

**AI Tool Usage:** Ask Copilot for launch_testing boilerplate

**Deliverable:** `colcon test` passes, integration test verifies end-to-end behavior

---

### HOUR 15 (2:00–3:00) — Extra Credit: Obstacle Avoidance
**Priority: BONUS (Extra Credit)**

**Tasks:**
- [ ] Add obstacle representation to the system:
  ```cpp
  struct Obstacle {
      double x, y, radius;  // circular obstacles
  };
  ```
- [ ] Implement collision checking against smoothed path
- [ ] Add simple obstacle avoidance:
  - **Option A: Potential Field Method** — Add repulsive potential around obstacles
  - **Option B: Path deformation** — Push path away from obstacles
  - **Option C: Use Dynamic Window Approach (DWA)** as local planner
- [ ] Create obstacle world SDF file for Gazebo testing
- [ ] Document the obstacle avoidance approach in README

**Reference:** `PythonRobotics/PathPlanning/DynamicWindowApproach/`

**AI Tool Usage:**
- Ask Copilot for potential field implementation
- Reference paper: "The Dynamic Window Approach to Collision Avoidance" (Fox et al.)

**Deliverable:** Robot avoids at least 2-3 static obstacles while tracking trajectory

---

### HOUR 16 (3:00–4:00) — Simulation Polish & Multiple Scenarios
**Priority: HIGH (20 pts Simulation)**

**Tasks:**
- [ ] Test and record 3-4 different scenarios:
  1. **Simple path**: 5 waypoints in open space
  2. **Complex path**: 10+ waypoints with sharp turns
  3. **Performance test**: Very close waypoints (stress test)
  4. **Obstacle avoidance**: Path through obstacles (if implemented)
- [ ] Ensure RViz2 displays:
  - Original waypoints (red spheres)
  - Smoothed path (green line)
  - Robot trajectory (blue line — actual path taken)
  - Tracking error (color-coded)
  - Velocity vectors
- [ ] Create a clean RViz2 config file saved to `config/rviz_config.rviz`
- [ ] Record rosbags for each scenario

**AI Tool Usage:** Use Foxglove Studio for advanced visualization if RViz2 is insufficient

**Deliverable:** Clean simulation with 3-4 scenarios working reliably

---

### HOUR 17 (4:00–5:00) — Generate Plots & Performance Analysis
**Priority: HIGH (Part of Documentation)**

**Tasks:**
- [ ] Generate publication-quality plots using Python/Matplotlib:
  1. **Path comparison**: Original waypoints vs smoothed path vs actual robot path
  2. **Tracking error over time**: Cross-track error (m) vs time (s)
  3. **Velocity profiles**: Commanded vs actual linear and angular velocity
  4. **Curvature analysis**: Path curvature before and after smoothing
  5. **Phase plot**: x-y trajectory with velocity magnitude color-coded
- [ ] Save all plots as high-resolution PNGs to `docs/figures/`
- [ ] Compute quantitative metrics:
  - Mean/max cross-track error
  - Mean/max heading error
  - Trajectory completion time
  - Smoothness metric (total curvature integral)

**AI Tool Usage:** Ask Copilot to generate matplotlib code for each plot type

**Deliverable:** 5+ professional plots saved in `docs/figures/`

---

### HOUR 18-19 (5:00–7:00) — Documentation: README & Design Report
**Priority: HIGH (15 pts)**

**Tasks:**
- [ ] Write comprehensive `README.md`:
  ```markdown
  # Path Smoothing & Trajectory Control for Differential Drive Robot
  
  ## Overview
  Brief description of the project
  
  ## System Architecture
  Architecture diagram (use Mermaid)
  
  ## Prerequisites
  - ROS2 Humble
  - Gazebo
  - TurtleBot3 packages
  - Eigen3, etc.
  
  ## Installation & Setup
  Step-by-step build instructions
  
  ## Running the Simulation
  Complete launch instructions
  
  ## Algorithm Design
  ### Path Smoothing
  - Cubic spline interpolation approach
  - Mathematical formulation
  - Why this over alternatives
  
  ### Trajectory Generation
  - Trapezoidal velocity profiling
  - Time parameterization
  
  ### Trajectory Tracking Controller
  - Pure Pursuit algorithm
  - PID speed control
  - Differential drive kinematics
  
  ## Results
  - Plots and analysis
  - Performance metrics table
  
  ## Testing
  - How to run tests
  - Test coverage summary
  
  ## Extension to Real Robot
  - Hardware considerations
  - Sensor integration
  - Localization requirements
  - Network latency considerations
  
  ## Obstacle Avoidance Extension
  - Approach description
  - Implementation details
  
  ## AI Tools Used
  - GitHub Copilot: Code generation, boilerplate, debugging
  - Claude: Algorithm verification, mathematical derivations
  - ChatGPT: ROS2 API questions
  - Perplexity: Research references
  
  ## References
  - Papers, textbooks, online resources
  ```

- [ ] Create `docs/DESIGN_DECISIONS.md`:
  - Why cubic spline over Bezier curves
  - Why Pure Pursuit over Stanley/MPC
  - Why trapezoidal velocity over S-curve
  - Architecture decisions (modularity, separation of concerns)

- [ ] Add architecture diagram:
  ```mermaid
  graph LR
      A[Waypoints YAML] --> B[Path Smoother]
      B --> C[Trajectory Generator]
      C --> D[Trajectory Controller]
      E[/odom] --> D
      D --> F[/cmd_vel]
      D --> G[/tracking_error]
      B --> H[/smoothed_path]
  ```

**AI Tool Usage:** Ask Copilot to help structure and write documentation

**Deliverable:** Complete README.md + Design document

---

### HOUR 20 (7:00–8:00) — Code Architecture Review & Comments
**Priority: MEDIUM (10 pts)**

**Tasks:**
- [ ] Review entire codebase for architecture quality:
  - Single Responsibility Principle (each class does one thing)
  - Dependency Injection (controller doesn't create smoother)
  - Interface segregation (abstract base classes for algorithms)
  - Configurable parameters (no magic numbers)
- [ ] Add comprehensive comments:
  - File-level docstrings explaining purpose
  - Class-level documentation
  - Method-level documentation with @param, @return
  - Inline comments for non-obvious logic
  - Algorithm step comments referencing equations/papers
- [ ] Add header guards, include what you use
- [ ] Run clang-format on all files
- [ ] Run cppcheck for static analysis
  ```bash
  cppcheck --enable=all --std=c++17 src/
  ```

**AI Tool Usage:** Ask Copilot to review code and suggest improvements

**Deliverable:** Clean, well-documented, architecturally sound codebase

---

### HOUR 21-22 (8:00–10:00) — Video Recording & Final Demo
**Priority: HIGH (Part of 15 pts)**

**Tasks:**
- [ ] Install OBS Studio if not already installed
- [ ] Plan video structure (3-5 minutes):
  ```
  0:00 - 0:30  Introduction & problem statement
  0:30 - 1:30  System architecture overview (show diagram)
  1:30 - 2:30  Live demo: Launch simulation, show robot tracking
  2:30 - 3:30  Show plots: tracking error, velocity profiles, path comparison
  3:30 - 4:00  Show test results (colcon test output)
  4:00 - 4:30  Obstacle avoidance demo (if implemented)
  4:30 - 5:00  Summary & design decisions
  ```
- [ ] Record screen with OBS Studio:
  - Show terminal commands
  - Show Gazebo simulation
  - Show RViz2 visualization
  - Show plots
  - Show test output
- [ ] Do 2-3 takes, pick the best one
- [ ] Trim/edit if needed (use Kdenlive, DaVinci Resolve, or CapCut)

**AI Tool Usage:** Use AI video editing tools if needed for quick cuts

**Deliverable:** 3-5 minute demonstration video (MP4)

---

### HOUR 23 (10:00–11:00) — Final Testing & Quality Assurance
**Priority: HIGH**

**Tasks:**
- [ ] Full clean build from scratch:
  ```bash
  cd ~/robotics_assignment_ws
  rm -rf build/ install/ log/
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  colcon test
  colcon test-result --verbose
  ```
- [ ] Verify ALL tests pass
- [ ] Test on a fresh terminal (source setup, run launch file)
- [ ] Check README instructions by following them step-by-step
- [ ] Verify all files are committed to git
- [ ] Check file structure matches submission requirements

**Deliverable:** Everything builds and runs from clean state

---

### HOUR 24 (11:00–12:00) — Final Packaging & Submission
**Priority: CRITICAL**

**Tasks:**
- [ ] Final git status check:
  ```bash
  git status
  git log --oneline -20
  ```
- [ ] Create release tag:
  ```bash
  git tag -a v1.0 -m "Assignment submission"
  git push origin main --tags
  ```
- [ ] Push to GitHub/GitLab repository
- [ ] Verify repository is accessible (if private, share with evaluator)
- [ ] Upload demo video (YouTube unlisted or Google Drive)
- [ ] Submit according to assignment instructions
- [ ] Double-check all submission requirements:
  - [x] Code Repository (well-documented, modular C++ ROS2)
  - [x] README with setup & execution instructions
  - [x] Design choices explained
  - [x] Real robot extension explained
  - [x] AI tools documented
  - [x] Obstacle avoidance extension documented
  - [x] Demo video (3-5 min)
  - [x] Plots and profiles

---

## FILE STRUCTURE AT SUBMISSION

```
robotics_assignment_ws/
├── src/
│   └── trajectory_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/trajectory_control/
│       │   ├── path_smoother.hpp
│       │   ├── trajectory_generator.hpp
│       │   ├── trajectory_controller.hpp
│       │   ├── obstacle_avoidance.hpp      (extra credit)
│       │   └── types.hpp
│       ├── src/
│       │   ├── path_smoother.cpp
│       │   ├── trajectory_generator.cpp
│       │   ├── trajectory_controller.cpp
│       │   ├── obstacle_avoidance.cpp       (extra credit)
│       │   └── trajectory_node.cpp
│       ├── launch/
│       │   ├── simulation.launch.py
│       │   └── trajectory_demo.launch.py
│       ├── config/
│       │   ├── waypoints.yaml
│       │   ├── controller_params.yaml
│       │   └── rviz_config.rviz
│       ├── test/
│       │   ├── test_path_smoother.cpp
│       │   ├── test_trajectory_generator.cpp
│       │   ├── test_controller.cpp
│       │   └── test_integration.py
│       ├── scripts/
│       │   ├── plot_trajectory.py
│       │   └── waypoint_publisher.py
│       └── worlds/
│           └── obstacle_world.sdf
├── docs/
│   ├── DESIGN_DECISIONS.md
│   └── figures/
│       ├── path_comparison.png
│       ├── velocity_profile.png
│       ├── tracking_error.png
│       ├── curvature_analysis.png
│       └── architecture_diagram.png
├── README.md
├── .gitignore
├── .clang-format
└── demo_video.mp4  (or link in README)
```

---

## KEY ALGORITHMS — QUICK REFERENCE LINKS

| Algorithm | Best Reference | Implementation Reference |
|---|---|---|
| Cubic Spline | [Wikipedia: Spline Interpolation](https://en.wikipedia.org/wiki/Spline_interpolation) | PythonRobotics: `PathPlanning/CubicSpline/` |
| Pure Pursuit | [CMU Pure Pursuit Paper](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) | PythonRobotics: `PathTracking/pure_pursuit/` |
| Trapezoidal Profile | [Motion Control Handbook](https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article) | Custom implementation |
| DWA (obstacle avoidance) | [Fox et al. 1997](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf) | PythonRobotics: `PathPlanning/DynamicWindowApproach/` |
| PID Control | [Wikipedia: PID Controller](https://en.wikipedia.org/wiki/PID_controller) | Standard implementation |
| Diff Drive Kinematics | [Introduction to Autonomous Mobile Robots](https://link.springer.com/book/10.1007/978-0-387-35301-8) | ROS2 TurtleBot3 model |

---

## RISK MITIGATION

| Risk | Mitigation | Fallback |
|---|---|---|
| ROS2/Gazebo installation issues | Use Docker: `ros:humble-desktop` | Use Python-only simulation (matplotlib animation) |
| TurtleBot3 packages not working | Switch to simple diff drive robot in Gazebo | Use custom URDF |
| Controller oscillation | Systematic parameter tuning, add damping | Switch to simpler P-controller |
| Time running out on obstacle avoidance | Document the approach even if not fully implemented | Submit without extra credit |
| Video recording issues | Use `ros2 bag record` + screen capture | Submit rosbag + screenshots |
| C++ compilation errors | Prototype in Python first, then port | Submit Python implementation with ROS2 |

---

## CRITICAL TIPS

1. **Build incrementally** — Compile after every small change, don't write 200 lines and then compile
2. **Test early, test often** — Run `colcon test` after each new test
3. **Git commit frequently** — At least every hour, with descriptive messages
4. **Use `ros2 topic echo`** — To debug topic communication issues
5. **Use `rqt_graph`** — To visualize node/topic connections
6. **Parameters in YAML** — Never hardcode; use ROS2 parameters
7. **Ask AI early** — Don't spend 30 min debugging what Copilot can spot in 10 seconds
8. **Prototype in Python** — If stuck on C++, get the algorithm working in Python first

---

*This plan was generated with GitHub Copilot assistance on Feb 25, 2026. Good luck!*
