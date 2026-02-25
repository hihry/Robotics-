/**
 * @file smoother_factory.hpp
 * @brief Factory for creating path smoother instances by config string.
 *
 * Strategy + Factory pattern — caller doesn't know which smoother is used.
 *
 * Usage:
 * @code
 *   auto smoother = SmootherFactory::create("cubic_spline");
 *   auto path = smoother->smooth(waypoints, 200);
 * @endcode
 */

#ifndef SMOOTH_NAV_CORE__PATH_SMOOTHER__SMOOTHER_FACTORY_HPP_
#define SMOOTH_NAV_CORE__PATH_SMOOTHER__SMOOTHER_FACTORY_HPP_

#include "smooth_nav_core/path_smoother/i_path_smoother.hpp"
#include "smooth_nav_core/path_smoother/cubic_spline_smoother.hpp"
#include "smooth_nav_core/path_smoother/bspline_smoother.hpp"
#include <memory>
#include <string>
#include <stdexcept>

namespace smooth_nav_core
{

class SmootherFactory
{
public:
  /**
   * @brief Create a path smoother by algorithm name.
   *
   * @param type "cubic_spline" or "bspline"
   * @return std::unique_ptr<IPathSmoother>
   * @throws std::invalid_argument for unknown type
   */
  static std::unique_ptr<IPathSmoother> create(const std::string & type)
  {
    if (type == "cubic_spline") {
      return std::make_unique<CubicSplineSmoother>();
    } else if (type == "bspline" || type == "gradient_descent") {
      return std::make_unique<BSplineSmoother>();
    } else {
      throw std::invalid_argument(
        "Unknown smoother type: '" + type + "'. Use 'cubic_spline' or 'bspline'.");
    }
  }
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__PATH_SMOOTHER__SMOOTHER_FACTORY_HPP_
