/**
 * @file types.hpp
 * @brief Common data types used across the trajectory control system.
 *
 * Defines Pose2D, TrajectoryPoint, and Obstacle structs shared by
 * the path smoother, trajectory generator, and controller modules.
 */

#ifndef TRAJECTORY_CONTROL__TYPES_HPP_
#define TRAJECTORY_CONTROL__TYPES_HPP_

#include <cmath>
#include <vector>
#include <string>

namespace trajectory_control
{

/**
 * @brief 2D pose with position and heading.
 */
struct Pose2D
{
  double x = 0.0;      ///< X position (meters)
  double y = 0.0;      ///< Y position (meters)
  double theta = 0.0;  ///< Heading angle (radians, [-pi, pi])

  Pose2D() = default;
  Pose2D(double x_, double y_, double theta_ = 0.0)
  : x(x_), y(y_), theta(theta_) {}

  /// Euclidean distance to another pose
  double distanceTo(const Pose2D & other) const
  {
    return std::hypot(x - other.x, y - other.y);
  }
};

/**
 * @brief A single point on a time-parameterized trajectory.
 *
 * Contains pose, velocities, curvature, and timestamp.
 */
struct TrajectoryPoint
{
  double x = 0.0;          ///< X position (m)
  double y = 0.0;          ///< Y position (m)
  double theta = 0.0;      ///< Heading (rad)
  double v = 0.0;          ///< Linear velocity (m/s)
  double omega = 0.0;      ///< Angular velocity (rad/s)
  double curvature = 0.0;  ///< Path curvature (1/m)
  double timestamp = 0.0;  ///< Time from trajectory start (s)
  double arc_length = 0.0; ///< Cumulative arc length (m)

  TrajectoryPoint() = default;
  TrajectoryPoint(double x_, double y_, double t_)
  : x(x_), y(y_), timestamp(t_) {}
};

/**
 * @brief Circular obstacle representation for collision checking.
 */
struct Obstacle
{
  double x = 0.0;       ///< Center X (m)
  double y = 0.0;       ///< Center Y (m)
  double radius = 0.5;  ///< Obstacle radius (m)

  Obstacle() = default;
  Obstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}

  /// Check if a point is inside the obstacle (with safety margin)
  bool contains(double px, double py, double margin = 0.0) const
  {
    double dist = std::hypot(px - x, py - y);
    return dist < (radius + margin);
  }
};

/**
 * @brief Velocity profile type for trajectory generation.
 */
enum class VelocityProfileType
{
  CONSTANT,     ///< Constant velocity throughout
  TRAPEZOIDAL   ///< Accelerate → cruise → decelerate
};

/**
 * @brief Smoothing algorithm type.
 */
enum class SmoothingMethod
{
  CUBIC_SPLINE,       ///< Cubic spline interpolation (C2 continuous)
  GRADIENT_DESCENT    ///< Iterative gradient descent smoothing
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TYPES_HPP_
