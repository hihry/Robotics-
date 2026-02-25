/**
 * @file trajectory_validator.hpp
 * @brief Helper assertions for validating trajectory properties in tests.
 */

#ifndef SMOOTH_NAV_CORE__TEST__FIXTURES__TRAJECTORY_VALIDATOR_HPP_
#define SMOOTH_NAV_CORE__TEST__FIXTURES__TRAJECTORY_VALIDATOR_HPP_

#include "smooth_nav_core/math/types.hpp"
#include <vector>
#include <cmath>

namespace smooth_nav_core::test
{

/**
 * @brief Check that trajectory timestamps are monotonically increasing.
 */
inline bool isTimeMonotonic(const std::vector<smooth_nav_core::TrajectoryPoint> & traj)
{
  for (size_t i = 1; i < traj.size(); ++i) {
    if (traj[i].timestamp < traj[i - 1].timestamp) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check that arc lengths are monotonically non-decreasing.
 */
inline bool isArcLengthMonotonic(const std::vector<smooth_nav_core::TrajectoryPoint> & traj)
{
  for (size_t i = 1; i < traj.size(); ++i) {
    if (traj[i].arc_length < traj[i - 1].arc_length - 1e-9) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check that velocities stay within bounds.
 */
inline bool velocitiesWithinBounds(
  const std::vector<smooth_nav_core::TrajectoryPoint> & traj,
  double v_max, double omega_max)
{
  for (const auto & pt : traj) {
    if (std::abs(pt.v) > v_max + 1e-6) return false;
    if (std::abs(pt.omega) > omega_max + 1e-6) return false;
  }
  return true;
}

/**
 * @brief Check that the last point has zero velocity (robot stops).
 */
inline bool endsWithZeroVelocity(const std::vector<smooth_nav_core::TrajectoryPoint> & traj)
{
  if (traj.empty()) return false;
  return std::abs(traj.back().v) < 1e-6;
}

/**
 * @brief Check path point sequence has monotonic arc lengths.
 */
inline bool pathArcLengthMonotonic(const std::vector<smooth_nav_core::PathPoint> & path)
{
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i].arc_length < path[i - 1].arc_length - 1e-9) {
      return false;
    }
  }
  return true;
}

}  // namespace smooth_nav_core::test

#endif  // SMOOTH_NAV_CORE__TEST__FIXTURES__TRAJECTORY_VALIDATOR_HPP_
