/**
 * @file i_trajectory_generator.hpp
 * @brief Abstract interface for trajectory generation algorithms.
 */

#ifndef SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__I_TRAJECTORY_GENERATOR_HPP_
#define SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__I_TRAJECTORY_GENERATOR_HPP_

#include "smooth_nav_core/math/types.hpp"
#include <vector>

namespace smooth_nav_core
{

class ITrajectoryGenerator
{
public:
  virtual ~ITrajectoryGenerator() = default;

  /**
   * @brief Generate a time-parameterized trajectory from a smooth path.
   *
   * @param path  Smooth path (PathPoint array). Must have >= 2 points.
   * @return std::vector<TrajectoryPoint> Time-stamped trajectory.
   */
  virtual std::vector<TrajectoryPoint> generate(
    const std::vector<PathPoint> & path) const = 0;

  virtual std::string name() const = 0;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__I_TRAJECTORY_GENERATOR_HPP_
