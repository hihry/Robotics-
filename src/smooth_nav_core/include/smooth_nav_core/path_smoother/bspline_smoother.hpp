/**
 * @file bspline_smoother.hpp
 * @brief B-spline path smoothing (bonus algorithm).
 *
 * Gradient-descent-based smoother that balances data fidelity with smoothness.
 * Based on Sebastian Thrun / Udacity approach.
 */

#ifndef SMOOTH_NAV_CORE__PATH_SMOOTHER__BSPLINE_SMOOTHER_HPP_
#define SMOOTH_NAV_CORE__PATH_SMOOTHER__BSPLINE_SMOOTHER_HPP_

#include "smooth_nav_core/path_smoother/i_path_smoother.hpp"
#include <vector>
#include <utility>

namespace smooth_nav_core
{

class BSplineSmoother : public IPathSmoother
{
public:
  BSplineSmoother() = default;
  ~BSplineSmoother() override = default;

  /**
   * @brief Set gradient descent weights and convergence parameters.
   */
  void setParams(double weight_data, double weight_smooth,
                 double tolerance, int max_iterations);

  std::vector<PathPoint> smooth(
    const std::vector<std::pair<double, double>> & waypoints,
    int num_points = 200) const override;

  std::string name() const override { return "bspline"; }

private:
  double weight_data_ = 0.1;
  double weight_smooth_ = 0.3;
  double tolerance_ = 1e-6;
  int max_iterations_ = 10000;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__PATH_SMOOTHER__BSPLINE_SMOOTHER_HPP_
