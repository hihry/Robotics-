/**
 * @file types.hpp
 * @brief Core data types used across smooth_nav algorithms.
 *
 * Pure C++17 — zero ROS dependencies.
 * All structs are value-semantic, trivially copyable, and constexpr-friendly.
 */

#ifndef SMOOTH_NAV_CORE__MATH__TYPES_HPP_
#define SMOOTH_NAV_CORE__MATH__TYPES_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <ostream>
#include <limits>

namespace smooth_nav_core
{

// ─────────────────────────────────────────────────────────────────────────────
// Robot physical / kinematic constraints
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Physical velocity and acceleration limits of a differential-drive robot.
 *
 * Default values match TurtleBot3 Burger (ROBOTIS spec-sheet).
 */
struct RobotLimits
{
  double max_linear_vel   = 0.22;   ///< m/s  (TurtleBot3 Burger: 0.22)
  double max_angular_vel  = 2.84;   ///< rad/s (TurtleBot3 Burger: 2.84)
  double max_linear_accel = 0.5;    ///< m/s²
  double max_angular_accel = 3.0;   ///< rad/s²
  double wheel_base       = 0.160;  ///< m — distance between wheels
  double wheel_radius     = 0.033;  ///< m
};

// ─────────────────────────────────────────────────────────────────────────────
// 2-D Geometry primitives
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Lightweight 2-D point (no heading).
 *
 * Useful where heading is irrelevant (spline evaluation, obstacle checks).
 */
struct Point2D
{
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  constexpr Point2D(double x_, double y_) : x(x_), y(y_) {}

  double distanceTo(const Point2D & o) const { return std::hypot(x - o.x, y - o.y); }

  Point2D operator+(const Point2D & o) const { return {x + o.x, y + o.y}; }
  Point2D operator-(const Point2D & o) const { return {x - o.x, y - o.y}; }
  Point2D operator*(double s)          const { return {x * s, y * s}; }

  bool operator==(const Point2D & o) const
  {
    return std::abs(x - o.x) < 1e-12 && std::abs(y - o.y) < 1e-12;
  }
  bool operator!=(const Point2D & o) const { return !(*this == o); }

  friend std::ostream & operator<<(std::ostream & os, const Point2D & p)
  {
    return os << "(" << p.x << ", " << p.y << ")";
  }
};

/**
 * @brief 2-D pose: position + heading (θ in radians, CCW from +x).
 */
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  Pose2D() = default;
  Pose2D(double x_, double y_, double theta_ = 0.0)
  : x(x_), y(y_), theta(theta_) {}

  double distanceTo(const Pose2D & other) const
  {
    return std::hypot(x - other.x, y - other.y);
  }

  Point2D position() const { return {x, y}; }

  bool operator==(const Pose2D & o) const
  {
    return std::abs(x - o.x) < 1e-12 &&
           std::abs(y - o.y) < 1e-12 &&
           std::abs(theta - o.theta) < 1e-12;
  }
  bool operator!=(const Pose2D & o) const { return !(*this == o); }

  friend std::ostream & operator<<(std::ostream & os, const Pose2D & p)
  {
    return os << "(x=" << p.x << ", y=" << p.y << ", θ=" << p.theta << ")";
  }
};

/**
 * @brief 2-D velocity (linear + angular) for a differential-drive robot.
 */
struct Velocity2D
{
  double linear  = 0.0;
  double angular = 0.0;

  Velocity2D() = default;
  Velocity2D(double v, double omega) : linear(v), angular(omega) {}

  friend std::ostream & operator<<(std::ostream & os, const Velocity2D & v)
  {
    return os << "(v=" << v.linear << ", ω=" << v.angular << ")";
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Path & trajectory data
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief A single point on a geometric path with curvature and arc length.
 */
struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;       ///< heading tangent to path
  double curvature = 0.0;   ///< signed curvature κ  (1/m)
  double arc_length = 0.0;  ///< cumulative arc length from path start

  PathPoint() = default;
  PathPoint(double x_, double y_, double theta_ = 0.0)
  : x(x_), y(y_), theta(theta_) {}

  Point2D position() const { return {x, y}; }

  friend std::ostream & operator<<(std::ostream & os, const PathPoint & p)
  {
    return os << "(x=" << p.x << ", y=" << p.y
              << ", θ=" << p.theta << ", κ=" << p.curvature
              << ", s=" << p.arc_length << ")";
  }
};

/**
 * @brief A single point on a time-parameterized trajectory.
 *
 * Extends PathPoint with velocity, angular velocity, and timestamp.
 */
struct TrajectoryPoint
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;           ///< linear velocity (m/s)
  double omega = 0.0;       ///< angular velocity (rad/s)
  double curvature = 0.0;   ///< signed curvature (1/m)
  double timestamp = 0.0;   ///< seconds from trajectory start
  double arc_length = 0.0;  ///< cumulative arc length

  TrajectoryPoint() = default;
  TrajectoryPoint(double x_, double y_, double t_)
  : x(x_), y(y_), timestamp(t_) {}

  /** @brief Convenience: construct from a PathPoint + velocity envelope. */
  explicit TrajectoryPoint(const PathPoint & pp, double v_ = 0.0,
                           double omega_ = 0.0, double t_ = 0.0)
  : x(pp.x), y(pp.y), theta(pp.theta), v(v_), omega(omega_),
    curvature(pp.curvature), timestamp(t_), arc_length(pp.arc_length) {}

  Point2D position() const { return {x, y}; }

  friend std::ostream & operator<<(std::ostream & os, const TrajectoryPoint & tp)
  {
    return os << "(x=" << tp.x << ", y=" << tp.y
              << ", v=" << tp.v << ", t=" << tp.timestamp << ")";
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Obstacles
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Circular obstacle for collision checking / distance queries.
 */
struct Obstacle
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.5;

  Obstacle() = default;
  Obstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}

  /** @brief True if point (px,py) is inside obstacle + optional margin. */
  bool contains(double px, double py, double margin = 0.0) const
  {
    return std::hypot(px - x, py - y) < (radius + margin);
  }

  /** @brief Signed distance: negative = inside, positive = outside. */
  double signedDistance(double px, double py) const
  {
    return std::hypot(px - x, py - y) - radius;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Controller diagnostics
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Snapshot of controller state — published for visualization / logging.
 */
struct ControllerState
{
  Pose2D robot_pose;
  Velocity2D command;
  double cross_track_error  = 0.0;
  double heading_error      = 0.0;
  size_t nearest_index      = 0;
  bool   trajectory_complete = false;
  double progress           = 0.0;   ///< [0,1] — fraction of trajectory completed
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__MATH__TYPES_HPP_
