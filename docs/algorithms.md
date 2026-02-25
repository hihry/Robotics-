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

### 2.2 Constant Velocity Profile

**Purpose:** Simple profile at fixed speed — useful for testing and comparison.

**Implementation:** `ConstantVelocityGenerator`.

---

## 3. Trajectory Tracking

### 3.1 Pure Pursuit Controller

**Purpose:** Compute steering commands to follow a trajectory using a look-ahead point.

**Algorithm:**
1. Find the nearest trajectory point to the robot's current position
2. Find the look-ahead point at distance $L_d$ ahead on the trajectory
3. Transform look-ahead point to robot's local frame
4. Compute curvature: $\kappa = \frac{2 y_{local}}{L_d^2}$
5. Angular velocity: $\omega = v \cdot \kappa$

### 3.2 PID Controller (heading correction)

Layered on top of Pure Pursuit for fine heading error correction:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de}{dt}$$

**Implementation:** `PurePursuitController` + `PIDController` (standalone).

---

## 4. Design Patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **Strategy** | `IPathSmoother`, `ITrajectoryGenerator`, `IController` | Swap algorithms at runtime |
| **Factory** | `SmootherFactory::create()` | Instantiate strategies by name/config |
| **Interface Segregation** | Separate interfaces per concern | Each node depends only on what it uses |
| **Config Over Code** | YAML parameters loaded at launch | Tune without recompiling |
