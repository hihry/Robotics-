/**
 * @file cubic_spline_smoother.cpp
 * @brief Implementation of natural cubic spline path smoothing.
 *
 * Uses Thomas algorithm tridiagonal solver from spline_math.hpp.
 * Guarantees C2-continuous output (continuous position, velocity, curvature).
 */

#include "smooth_nav_core/path_smoother/cubic_spline_smoother.hpp"
#include "smooth_nav_core/math/geometry_utils.hpp"
#include <cmath>
#include <stdexcept>

namespace smooth_nav_core
{

std::vector<PathPoint> CubicSplineSmoother::smooth(
  const std::vector<std::pair<double, double>> & waypoints,
  int num_points) const
{
  // --- Input validation ---
  if (waypoints.size() < 2) {
    throw std::invalid_argument(
      "CubicSplineSmoother::smooth() requires >= 2 waypoints, got " +
      std::to_string(waypoints.size()));
  }
  if (num_points < 2) {
    throw std::invalid_argument("Number of output points must be >= 2");
  }
  for (const auto & wp : waypoints) {
    if (std::isnan(wp.first) || std::isnan(wp.second) ||
        std::isinf(wp.first) || std::isinf(wp.second))
    {
      throw std::invalid_argument("Waypoints contain NaN or Inf values");
    }
  }

  const size_t n = waypoints.size();

  // Special case: exactly 2 points → linear interpolation
  if (n == 2) {
    std::vector<PathPoint> result;
    result.reserve(num_points);
    double theta = std::atan2(waypoints[1].second - waypoints[0].second,
                              waypoints[1].first - waypoints[0].first);
    for (int i = 0; i < num_points; ++i) {
      double t = static_cast<double>(i) / (num_points - 1);
      PathPoint pt;
      pt.x = waypoints[0].first + t * (waypoints[1].first - waypoints[0].first);
      pt.y = waypoints[0].second + t * (waypoints[1].second - waypoints[0].second);
      pt.theta = theta;
      pt.curvature = 0.0;  // straight line
      pt.arc_length = t * dist(waypoints[0].first, waypoints[0].second,
                                waypoints[1].first, waypoints[1].second);
      result.push_back(pt);
    }
    return result;
  }

  // Step 1: Compute cumulative arc lengths as parameter
  std::vector<double> arc = computeArcLengths(waypoints);

  // Step 2: Extract x and y coordinate arrays
  std::vector<double> wx(n), wy(n);
  for (size_t i = 0; i < n; ++i) {
    wx[i] = waypoints[i].first;
    wy[i] = waypoints[i].second;
  }

  // Step 3: Solve natural cubic splines for x(s) and y(s)
  SplineCoefficients coeff_x = solveNaturalCubicSpline(arc, wx);
  SplineCoefficients coeff_y = solveNaturalCubicSpline(arc, wy);

  // Step 4: Sample the spline at uniform arc-length intervals
  double total_length = arc.back();
  std::vector<PathPoint> result;
  result.reserve(num_points);

  double cumulative_arc = 0.0;

  for (int i = 0; i < num_points; ++i) {
    double s = total_length * static_cast<double>(i) / (num_points - 1);
    double x = evaluateSpline(s, arc, coeff_x);
    double y = evaluateSpline(s, arc, coeff_y);

    // Compute heading from path tangent
    double theta = 0.0;
    if (i < num_points - 1) {
      double s_next = total_length * static_cast<double>(i + 1) / (num_points - 1);
      double x_next = evaluateSpline(s_next, arc, coeff_x);
      double y_next = evaluateSpline(s_next, arc, coeff_y);
      theta = std::atan2(y_next - y, x_next - x);
    } else if (!result.empty()) {
      theta = result.back().theta;
    }

    PathPoint pt;
    pt.x = x;
    pt.y = y;
    pt.theta = theta;
    pt.arc_length = s;
    pt.curvature = 0.0;  // will be computed below
    result.push_back(pt);
  }

  // Step 5: Compute curvature at each point via central differences
  for (size_t i = 1; i + 1 < result.size(); ++i) {
    double dx = (result[i + 1].x - result[i - 1].x) / 2.0;
    double dy = (result[i + 1].y - result[i - 1].y) / 2.0;
    double ddx = result[i + 1].x - 2.0 * result[i].x + result[i - 1].x;
    double ddy = result[i + 1].y - 2.0 * result[i].y + result[i - 1].y;
    double denom = std::pow(dx * dx + dy * dy, 1.5);
    if (denom > 1e-12) {
      result[i].curvature = (dx * ddy - dy * ddx) / denom;
    }
  }

  return result;
}

}  // namespace smooth_nav_core
