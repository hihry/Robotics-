# Algorithms — smooth_nav

This document describes the mathematical algorithms implemented in `smooth_nav_core`.

---

## 1. Path Smoothing

### 1.1 Natural Cubic Spline

**Purpose:** Given a set of waypoints, produce a C²-continuous curve that passes exactly through every waypoint.

**Mathematical Formulation:**

Between consecutive knots $(x_i, y_i)$ and $(x_{i+1}, y_{i+1})$, we fit a cubic polynomial:

$$S_i(t) = a_i + b_i(t - t_i) + c_i(t - t_i)^2 + d_i(t - t_i)^3$$

where $t$ is the cumulative arc-length parameter.

**Boundary conditions:** Natural spline → $S''(t_0) = S''(t_n) = 0$.

**Solution method:** Thomas algorithm (tridiagonal matrix solver) — $O(n)$ time and space.

**Implementation:** `spline_math.hpp` → `solveNaturalCubicSpline()`, used by `CubicSplineSmoother`.

---

### 1.2 B-Spline Gradient Descent Smoother

**Purpose:** Iteratively minimize a cost function that balances path smoothness against fidelity to original waypoints.

**Cost function:**

$$J = \alpha \sum_{i} \|\mathbf{p}_i - \mathbf{w}_i\|^2 + \beta \sum_{i} \|\mathbf{p}_{i-1} - 2\mathbf{p}_i + \mathbf{p}_{i+1}\|^2$$

- First term: data fidelity (stay near original waypoints)
- Second term: smoothness (minimize second finite differences ≈ curvature)

**Parameters:** `weight_smooth` ($\beta$), `weight_data` ($\alpha$), `learning_rate`, `iterations`.

**Implementation:** `BSplineSmoother` iteratively updates interior control points.

---

## 2. Trajectory Generation

### 2.1 Trapezoidal Velocity Profile

**Purpose:** Generate a time-parameterized trajectory with acceleration, cruise, and deceleration phases.

**Three phases:**

1. **Acceleration:** $v(s) = \sqrt{2 a_{max} \cdot s}$ for $s \in [0, s_{accel}]$
2. **Cruise:** $v(s) = v_{max}$ for $s \in [s_{accel}, s_{total} - s_{decel}]$
3. **Deceleration:** $v(s) = \sqrt{2 a_{max} \cdot (s_{total} - s)}$ for $s \in [s_{total} - s_{decel}, s_{total}]$

Where $s_{accel} = s_{decel} = \frac{v_{max}^2}{2 a_{max}}$.

**Triangular fallback:** If the path is too short for full cruise, a triangular profile is used where the robot accelerates then immediately decelerates.

**Implementation:** `TrapezoidalVelocityGenerator`.

### 2.3 Curvature-Based Speed Limiting

**Purpose:** Prevent the robot from taking sharp turns at high speed, which causes wheel slip and tracking error.

**Constraint:** At any point with curvature $\kappa$, the lateral (centripetal) acceleration must not exceed a limit:

$$a_{lat} = v^2 \cdot |\kappa| \leq a_{lat,max}$$

Solving for the curvature-limited speed:

$$v_{curv} = \sqrt{\frac{a_{lat,max}}{|\kappa|}}$$

The actual velocity at each point is the **minimum** of the trapezoidal profile speed and the curvature limit:

$$v(s) = \min\left(v_{trap}(s),\ \sqrt{\frac{a_{lat,max}}{|\kappa(s)|}}\right)$$

**Parameter:** `max_lateral_acceleration` (default: 0.5 m/s²).

**Implementation:** Applied inside both `TrapezoidalVelocityGenerator` and `ConstantVelocityGenerator` via `setMaxLateralAcceleration()`.

### 2.4 Constant Velocity Profile

**Purpose:** Simple profile at fixed speed — useful for testing and comparison.

**Implementation:** `ConstantVelocityGenerator`.

---

## 3. Trajectory Tracking

### 3.1 Pure Pursuit Controller

**Purpose:** Compute steering commands to follow a trajectory using a look-ahead point.

**Algorithm:**
1. Find the nearest trajectory point to the robot's current position
2. Compute adaptive look-ahead distance: $L_d = L_{d,base} + k \cdot |v|$
3. Find the look-ahead point at distance $L_d$ ahead on the trajectory
4. Transform look-ahead point to robot's local frame
5. Compute curvature: $\kappa = \frac{2 y_{local}}{L_d^2}$
6. Angular velocity: $\omega = v \cdot \kappa$

### 3.2 Adaptive Look-Ahead

**Purpose:** At low speed the robot needs tight tracking (small $L_d$); at high speed a larger $L_d$ prevents oscillation.

$$L_d = L_{d,base} + k_{ld} \cdot |v|$$

**Parameters:** `look_ahead_distance` ($L_{d,base}$, default 0.3 m), `adaptive_look_ahead_gain` ($k_{ld}$, default 0.5). Set $k_{ld} = 0$ for classic fixed look-ahead.

### 3.3 Goal Deceleration

**Purpose:** Prevent overshoot at the final waypoint by linearly reducing speed inside a deceleration radius.

$$v_{cmd} = v_{ref} \cdot \min\left(1,\ \frac{d_{goal}}{r_{decel}}\right)$$

where $d_{goal}$ is the distance to the last trajectory point and $r_{decel}$ is the deceleration radius (default 0.3 m).

**Implementation:** Inside `PurePursuitController::computeControl()`.

### 3.4 PID Controller (cross-track correction)

Layered on top of Pure Pursuit for fine cross-track error correction:

$$\omega_{correction} = K_p \cdot e_{cte} + K_i \int_0^t e_{cte}(\tau) d\tau + K_d \frac{de_{cte}}{dt}$$

The signed cross-track error is computed via the cross product of the path tangent and the robot-to-path vector, providing directionality (left vs. right of path).

**Implementation:** `PurePursuitController` + `PIDController` (standalone, composable).

---

## 4. Safety Layer

### 4.1 Velocity & Acceleration Limiting

The `safety_watchdog_node` sits between the controller (`/cmd_vel_raw`) and the motor driver (`/cmd_vel`). It enforces:

$$|v_{cmd}| \leq v_{max},\quad |\omega_{cmd}| \leq \omega_{max}$$

$$\frac{|v_{cmd} - v_{prev}|}{\Delta t} \leq a_{max}$$

### 4.2 Command Timeout (Watchdog)

If no `/cmd_vel_raw` message is received for `cmd_vel_timeout` seconds (default 0.5), the watchdog publishes a zero-velocity command — an emergency stop.

### 4.3 Laser Proximity Stop (optional)

When `use_laser_safety` is enabled, the watchdog subscribes to `/scan` and halts the robot if any range reading falls below `obstacle_stop_distance` (default 0.20 m) in the forward arc (±45°).

---

## 5. Design Patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **Strategy** | `IPathSmoother`, `ITrajectoryGenerator`, `IController` | Swap algorithms at runtime |
| **Factory** | `SmootherFactory::create()` | Instantiate strategies by name/config |
| **Pipeline Orchestrator** | `waypoint_client_node` | Sequences Smooth → Generate → Execute end-to-end |
| **Safety Interposer** | `safety_watchdog_node` | Transparent filtering between controller and actuators |
| **Interface Segregation** | Separate interfaces per concern | Each node depends only on what it uses |
| **Config Over Code** | YAML parameters loaded at launch | Tune without recompiling |
| **Dynamic Reconfiguration** | All C++ nodes | `add_on_set_parameters_callback` for live tuning |
