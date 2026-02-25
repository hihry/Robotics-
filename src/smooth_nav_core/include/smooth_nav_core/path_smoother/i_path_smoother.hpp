/**
 * @file i_path_smoother.hpp
 * @brief Abstract interface for path smoothing algorithms (Strategy pattern).
 *
 * All path smoothers implement this interface, allowing runtime swapping
 * via SmootherFactory.
 */

#ifndef SMOOTH_NAV_CORE__PATH_SMOOTHER__I_PATH_SMOOTHER_HPP_
#define SMOOTH_NAV_CORE__PATH_SMOOTHER__I_PATH_SMOOTHER_HPP_

#include "smooth_nav_core/math/types.hpp"
#include <vector>
#include <utility>

namespace smooth_nav_core
{

/**
 * @class IPathSmoother
 * @brief Abstract interface for path smoothing algorithms.
 */
class IPathSmoother
{
public:
  virtual ~IPathSmoother() = default;

  /**
   * @brief Smooth a set of waypoints into a dense, continuous path.
   *
   * @param waypoints Input waypoints as (x, y) pairs. Must have >= 2 points.
   * @param num_points Number of desired output points.
   * @return std::vector<PathPoint> Smoothed path with heading and curvature.
   * @throws std::invalid_argument if waypoints has fewer than 2 points.
   */
  virtual std::vector<PathPoint> smooth(
    const std::vector<std::pair<double, double>> & waypoints,
    int num_points = 200) const = 0;

  /**
   * @brief Get the name of this smoother algorithm.
   */
  virtual std::string name() const = 0;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__PATH_SMOOTHER__I_PATH_SMOOTHER_HPP_
