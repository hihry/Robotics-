/**
 * @file path_smoother.hpp
 * @brief Path smoothing algorithms for converting discrete waypoints
 *        into smooth, continuous paths.
 *
 * Provides two smoothing methods:
 *   1. Cubic Spline Interpolation — C2 continuous, parameterized by arc length
 *   2. Gradient Descent Smoothing — iterative optimization balancing data
 *      fidelity and smoothness
 *
 * Reference:
 *   - Cubic Spline: https://en.wikipedia.org/wiki/Spline_interpolation
 *   - PythonRobotics: PathPlanning/CubicSpline/cubic_spline_planner.py
 */

#ifndef TRAJECTORY_CONTROL__PATH_SMOOTHER_HPP_
#define TRAJECTORY_CONTROL__PATH_SMOOTHER_HPP_

#include "trajectory_control/types.hpp"
#include <vector>
#include <stdexcept>

namespace trajectory_control
{

/**
 * @class PathSmoother
 * @brief Generates a smooth, dense path from a sparse set of 2D waypoints.
 *
 * Usage:
 * @code
 *   PathSmoother smoother;
 *   smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
 *   smoother.setNumPoints(200);
 *   auto smooth_path = smoother.smooth(waypoints);
 * @endcode
 */
class PathSmoother
{
public:
  PathSmoother();
  ~PathSmoother() = default;

  // ---- Configuration ----

  /// Set the smoothing algorithm to use
  void setMethod(SmoothingMethod method);

  /// Set the number of output points on the smoothed path
  void setNumPoints(int num_points);

  /// Set gradient descent parameters (only used for GRADIENT_DESCENT method)
  void setGradientDescentParams(double weight_data, double weight_smooth,
                                 double tolerance, int max_iterations);

  // ---- Core API ----

  /**
   * @brief Smooth a set of waypoints into a dense, continuous path.
   *
   * @param waypoints Input waypoints as (x, y) pairs. Must have >= 2 points.
   * @return std::vector<Pose2D> Smoothed path with heading computed from tangent.
   * @throws std::invalid_argument if waypoints has fewer than 2 points.
   */
  std::vector<Pose2D> smooth(const std::vector<std::pair<double, double>> & waypoints) const;

  // ---- Individual Algorithms (public for testing) ----

  /// Cubic spline interpolation
  std::vector<Pose2D> cubicSplineSmooth(
    const std::vector<std::pair<double, double>> & waypoints) const;

  /// Gradient descent path smoothing
  std::vector<Pose2D> gradientDescentSmooth(
    const std::vector<std::pair<double, double>> & waypoints) const;

private:
  SmoothingMethod method_ = SmoothingMethod::CUBIC_SPLINE;
  int num_points_ = 200;  ///< Number of output samples

  // Gradient descent parameters
  double weight_data_ = 0.1;      ///< Data fidelity weight (alpha)
  double weight_smooth_ = 0.3;    ///< Smoothness weight (beta)
  double tolerance_ = 1e-6;       ///< Convergence tolerance
  int max_iterations_ = 10000;    ///< Max gradient descent iterations

  /**
   * @brief Compute cumulative arc length along waypoints.
   * @param waypoints The input waypoint list.
   * @return Vector of cumulative distances (first element = 0.0).
   */
  std::vector<double> computeArcLengths(
    const std::vector<std::pair<double, double>> & waypoints) const;

  /**
   * @brief Solve a natural cubic spline for 1D data.
   *
   * Given parameter values t[] and data values y[], compute spline
   * coefficients a, b, c, d for each segment.
   *
   * @param t Parameter values (e.g., arc lengths)
   * @param y Data values (e.g., x-coordinates or y-coordinates)
   * @param a Output: constant coefficients
   * @param b Output: linear coefficients
   * @param c Output: quadratic coefficients
   * @param d Output: cubic coefficients
   */
  void solveNaturalCubicSpline(
    const std::vector<double> & t,
    const std::vector<double> & y,
    std::vector<double> & a,
    std::vector<double> & b,
    std::vector<double> & c,
    std::vector<double> & d) const;

  /**
   * @brief Evaluate a cubic spline at a given parameter value.
   */
  double evaluateSpline(
    double t_eval,
    const std::vector<double> & t,
    const std::vector<double> & a,
    const std::vector<double> & b,
    const std::vector<double> & c,
    const std::vector<double> & d) const;
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__PATH_SMOOTHER_HPP_
