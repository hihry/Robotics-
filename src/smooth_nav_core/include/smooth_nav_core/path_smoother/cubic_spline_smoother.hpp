/**
 * @file cubic_spline_smoother.hpp
 * @brief Cubic spline path smoothing — C2 continuous.
 *
 * Natural cubic spline via Thomas algorithm tridiagonal solver.
 * Guarantees continuous first and second derivatives (curvature).
 */

#ifndef SMOOTH_NAV_CORE__PATH_SMOOTHER__CUBIC_SPLINE_SMOOTHER_HPP_
#define SMOOTH_NAV_CORE__PATH_SMOOTHER__CUBIC_SPLINE_SMOOTHER_HPP_

#include "smooth_nav_core/path_smoother/i_path_smoother.hpp"
#include "smooth_nav_core/math/spline_math.hpp"
#include <vector>
#include <utility>

namespace smooth_nav_core
{

class CubicSplineSmoother : public IPathSmoother
{
public:
  CubicSplineSmoother() = default;
  ~CubicSplineSmoother() override = default;

  std::vector<PathPoint> smooth(
    const std::vector<std::pair<double, double>> & waypoints,
    int num_points = 200) const override;

  std::string name() const override { return "cubic_spline"; }
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__PATH_SMOOTHER__CUBIC_SPLINE_SMOOTHER_HPP_
