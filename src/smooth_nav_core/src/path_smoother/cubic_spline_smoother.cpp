/**
 * @file cubic_spline_smoother.cpp
 * @brief Natural cubic spline path smoothing — C² continuous.
 *
 * Uses the Thomas-algorithm tridiagonal solver from spline_math.hpp,
 * and **analytic** first / second derivatives for heading and curvature
 * (no finite-difference approximation).
 *
 * Guarantees:
 *   • Output starts exactly at the first waypoint, ends at the last.
 *   • Continuous position, velocity (tangent), and curvature.
 *   • Arc-length values are monotonically non-decreasing.
 *
 * Complexity: O(n) spline solve + O(num_points) sampling.
 */

#include "smooth_nav_core/path_smoother/cubic_spline_smoother.hpp"
#include "smooth_nav_core/math/geometry_utils.hpp"
#include <cmath>
#include <stdexcept>

namespace smooth_nav_core
{

std::vector<PathPoint> CubicSplineSmoother::smooth(
  const std::vector<std::pair<double, double>> & raw_waypoints,
  int num_points) const
{
  // ────────────────────── Input validation ──────────────────────
  if (raw_waypoints.size() < 2) {
    throw std::invalid_argument(
      "CubicSplineSmoother::smooth() requires >= 2 waypoints, got " +
      std::to_string(raw_waypoints.size()));
  }
  if (num_points < 2) {
    throw std::invalid_argument("Number of output points must be >= 2");
  }
  for (const auto & wp : raw_waypoints) {
    if (std::isnan(wp.first) || std::isnan(wp.second) ||
        std::isinf(wp.first) || std::isinf(wp.second))
    {
      throw std::invalid_argument("Waypoints contain NaN or Inf values");
    }
  }

  // Remove consecutive duplicates (zero-length segments break the solver)
  auto waypoints = removeDuplicates(raw_waypoints);
  if (waypoints.size() < 2) {
    throw std::invalid_argument(
      "After duplicate removal, fewer than 2 unique waypoints remain");
  }

  const size_t n = waypoints.size();

  // ────────────────── Special case: 2 points (linear) ──────────────────
  if (n == 2) {
    std::vector<PathPoint> result;
    result.reserve(num_points);
    double theta = std::atan2(waypoints[1].second - waypoints[0].second,
                              waypoints[1].first  - waypoints[0].first);
    double total_d = dist(waypoints[0].first, waypoints[0].second,
                          waypoints[1].first, waypoints[1].second);
    for (int i = 0; i < num_points; ++i) {
      double t = static_cast<double>(i) / (num_points - 1);
      PathPoint pt;
      pt.x = lerp(waypoints[0].first,  waypoints[1].first,  t);
      pt.y = lerp(waypoints[0].second, waypoints[1].second, t);
      pt.theta      = theta;
      pt.curvature   = 0.0;
      pt.arc_length  = t * total_d;
      result.push_back(pt);
    }
    return result;
  }

  // ────────────────── Step 1: cumulative arc-length parameter ──────────────
  std::vector<double> arc = computeArcLengths(waypoints);
  double total_length = arc.back();
  if (total_length < 1e-12) {
    throw std::invalid_argument("Path has zero length — all waypoints are coincident");
  }

  // ────────────────── Step 2: coordinate arrays ──────────────
  std::vector<double> wx(n), wy(n);
  for (size_t i = 0; i < n; ++i) {
    wx[i] = waypoints[i].first;
    wy[i] = waypoints[i].second;
  }

  // ────────────────── Step 3: solve natural cubic splines x(s), y(s) ──────
  SplineCoefficients coeff_x = solveNaturalCubicSpline(arc, wx);
  SplineCoefficients coeff_y = solveNaturalCubicSpline(arc, wy);

  // ────────────────── Step 4: uniform arc-length sampling ──────────────────
  std::vector<PathPoint> result;
  result.reserve(num_points);

  for (int i = 0; i < num_points; ++i) {
    double s = total_length * static_cast<double>(i) / (num_points - 1);

    PathPoint pt;
    pt.x = evaluateSpline(s, arc, coeff_x);
    pt.y = evaluateSpline(s, arc, coeff_y);

    // Exact heading from analytic tangent: θ = atan2(y'(s), x'(s))
    pt.theta = computeExactHeading(s, arc, coeff_x, coeff_y);

    // Exact curvature: κ = (x'y'' − y'x'') / (x'² + y'²)^(3/2)
    pt.curvature = computeExactCurvature(s, arc, coeff_x, coeff_y);

    pt.arc_length = s;
    result.push_back(pt);
  }

  // Ensure endpoints match waypoints exactly (eliminate floating-point drift)
  result.front().x = waypoints.front().first;
  result.front().y = waypoints.front().second;
  result.back().x  = waypoints.back().first;
  result.back().y  = waypoints.back().second;

  return result;
}

}  // namespace smooth_nav_core
