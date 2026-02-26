/**
 * @file bspline_smoother.hpp
 * @brief Gradient-descent path smoother (bonus / extra credit algorithm).
 *
 * Optimises a cost function that balances three terms:
 *   1. **Data fidelity** — keeps the path close to the original waypoints.
 *   2. **Smoothness**    — minimises discrete second differences (curvature proxy).
 *   3. **Obstacle avoidance** (optional) — pushes the path away from obstacles.
 *
 * Inspired by Sebastian Thrun / Udacity robot path-smoothing approach.
 * After gradient descent, the optimised control points are up-sampled with
 * a natural cubic spline (C² continuous output), not plain linear interpolation.
 */

#ifndef SMOOTH_NAV_CORE__PATH_SMOOTHER__BSPLINE_SMOOTHER_HPP_
#define SMOOTH_NAV_CORE__PATH_SMOOTHER__BSPLINE_SMOOTHER_HPP_

#include "smooth_nav_core/path_smoother/i_path_smoother.hpp"
#include "smooth_nav_core/math/types.hpp"
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
   * @brief Set gradient-descent tuning parameters.
   *
   * @param weight_data    α — pull toward original waypoints.
   * @param weight_smooth  β — penalise second-difference (curvature).
   * @param tolerance      Convergence threshold on total change.
   * @param max_iterations Upper bound on iterations.
   */
  void setParams(double weight_data, double weight_smooth,
                 double tolerance, int max_iterations);

  /**
   * @brief (Optional) set obstacles for repulsive-field obstacle avoidance.
   *
   * During gradient descent, each point receives a repulsive gradient
   * proportional to 1/d² when closer than `safety_margin` to any obstacle.
   */
  void setObstacles(const std::vector<Obstacle> & obstacles,
                    double weight_obstacle = 0.5,
                    double safety_margin = 0.3);

  std::vector<PathPoint> smooth(
    const std::vector<std::pair<double, double>> & waypoints,
    int num_points = 200) const override;

  std::string name() const override { return "bspline"; }

private:
  double weight_data_   = 0.1;
  double weight_smooth_ = 0.3;
  double tolerance_      = 1e-6;
  int    max_iterations_ = 10000;

  // Optional obstacle avoidance
  std::vector<Obstacle> obstacles_;
  double weight_obstacle_ = 0.0;
  double safety_margin_   = 0.3;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__PATH_SMOOTHER__BSPLINE_SMOOTHER_HPP_
