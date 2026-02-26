/**
 * @file sample_waypoints.hpp
 * @brief Shared test waypoint sets for smooth_nav_core unit tests.
 */

#ifndef SMOOTH_NAV_CORE__TEST__FIXTURES__SAMPLE_WAYPOINTS_HPP_
#define SMOOTH_NAV_CORE__TEST__FIXTURES__SAMPLE_WAYPOINTS_HPP_

#include <vector>
#include <utility>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace smooth_nav_core::test
{

/**
 * @brief Straight line from (0,0) to (2,0).
 */
inline std::vector<std::pair<double, double>> straightLine()
{
  return {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
}

/**
 * @brief L-shaped path with a right-angle turn.
 */
inline std::vector<std::pair<double, double>> lShape()
{
  return {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {2.0, 2.0}};
}

/**
 * @brief S-curve path.
 */
inline std::vector<std::pair<double, double>> sCurve()
{
  return {{0.0, 0.0}, {0.5, 0.5}, {1.0, 0.0}, {1.5, -0.5}, {2.0, 0.0}};
}

/**
 * @brief Circular path (quarter circle approximated with waypoints).
 */
inline std::vector<std::pair<double, double>> quarterCircle()
{
  std::vector<std::pair<double, double>> pts;
  for (int i = 0; i <= 8; ++i) {
    double angle = M_PI / 2.0 * static_cast<double>(i) / 8.0;
    pts.emplace_back(std::cos(angle), std::sin(angle));
  }
  return pts;
}

/**
 * @brief Two-point path for edge-case testing.
 */
inline std::vector<std::pair<double, double>> twoPoints()
{
  return {{0.0, 0.0}, {1.0, 1.0}};
}

/**
 * @brief Dense square waypoints (useful for controller tests).
 */
inline std::vector<std::pair<double, double>> squarePath()
{
  return {
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0},
    {2.0, 1.0}, {2.0, 2.0},
    {1.0, 2.0}, {0.0, 2.0},
    {0.0, 1.0}, {0.0, 0.0}
  };
}

}  // namespace smooth_nav_core::test

#endif  // SMOOTH_NAV_CORE__TEST__FIXTURES__SAMPLE_WAYPOINTS_HPP_
