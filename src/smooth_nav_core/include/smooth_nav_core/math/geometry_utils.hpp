/**
 * @file geometry_utils.hpp
 * @brief Geometry utility functions — pure C++, no ROS.
 */

#ifndef SMOOTH_NAV_CORE__MATH__GEOMETRY_UTILS_HPP_
#define SMOOTH_NAV_CORE__MATH__GEOMETRY_UTILS_HPP_

#include <cmath>
#include <vector>
#include <utility>

namespace smooth_nav_core
{

/**
 * @brief Euclidean distance between two 2D points.
 */
inline double dist(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

/**
 * @brief Normalize angle to [-pi, pi].
 */
inline double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief Linear interpolation between two values.
 */
inline double lerp(double a, double b, double t)
{
  return a + t * (b - a);
}

/**
 * @brief Compute cumulative arc lengths for a sequence of (x,y) pairs.
 */
inline std::vector<double> computeArcLengths(
  const std::vector<std::pair<double, double>> & points)
{
  std::vector<double> arc(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    double dx = points[i].first - points[i - 1].first;
    double dy = points[i].second - points[i - 1].second;
    arc[i] = arc[i - 1] + std::hypot(dx, dy);
  }
  return arc;
}

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__MATH__GEOMETRY_UTILS_HPP_
