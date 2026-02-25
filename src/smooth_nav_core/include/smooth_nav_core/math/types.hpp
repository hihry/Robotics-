/**
 * @file types.hpp
 * @brief Core data types used across smooth_nav algorithms.
 *
 * Pure C++ — zero ROS dependencies.
 */

#ifndef SMOOTH_NAV_CORE__MATH__TYPES_HPP_
#define SMOOTH_NAV_CORE__MATH__TYPES_HPP_

#include <cmath>
#include <vector>
#include <string>

namespace smooth_nav_core
{

/**
 * @brief 2D pose with position and heading.
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
};

/**
 * @brief 2D velocity (linear + angular).
 */
struct Velocity2D
{
  double linear = 0.0;
  double angular = 0.0;

  Velocity2D() = default;
  Velocity2D(double v, double omega) : linear(v), angular(omega) {}
};

/**
 * @brief A single point on a geometric path with curvature.
 */
struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double curvature = 0.0;
  double arc_length = 0.0;

  PathPoint() = default;
  PathPoint(double x_, double y_, double theta_ = 0.0)
  : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief A single point on a time-parameterized trajectory.
 */
struct TrajectoryPoint
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;
  double omega = 0.0;
  double curvature = 0.0;
  double timestamp = 0.0;
  double arc_length = 0.0;

  TrajectoryPoint() = default;
  TrajectoryPoint(double x_, double y_, double t_)
  : x(x_), y(y_), timestamp(t_) {}
};

/**
 * @brief Circular obstacle representation for collision checking.
 */
struct Obstacle
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.5;

  Obstacle() = default;
  Obstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}

  bool contains(double px, double py, double margin = 0.0) const
  {
    return std::hypot(px - x, py - y) < (radius + margin);
  }
};

/**
 * @brief Controller state snapshot for diagnostics.
 */
struct ControllerState
{
  Pose2D robot_pose;
  Velocity2D command;
  double cross_track_error = 0.0;
  double heading_error = 0.0;
  size_t nearest_index = 0;
  bool trajectory_complete = false;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__MATH__TYPES_HPP_
