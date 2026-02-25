/**
 * @file bspline_smoother.cpp
 * @brief Gradient-descent path smoothing implementation.
 *
 * Minimizes: F = α||new - original||² + β||Δ²path||²
 * Based on Sebastian Thrun / Udacity smoothing algorithm.
 */

#include "smooth_nav_core/path_smoother/bspline_smoother.hpp"
#include "smooth_nav_core/math/geometry_utils.hpp"
#include <cmath>
#include <stdexcept>

namespace smooth_nav_core
{

void BSplineSmoother::setParams(
  double weight_data, double weight_smooth,
  double tolerance, int max_iterations)
{
  weight_data_ = weight_data;
  weight_smooth_ = weight_smooth;
  tolerance_ = tolerance;
  max_iterations_ = max_iterations;
}

std::vector<PathPoint> BSplineSmoother::smooth(
  const std::vector<std::pair<double, double>> & waypoints,
  int num_points) const
{
  // --- Input validation ---
  if (waypoints.size() < 2) {
    throw std::invalid_argument(
      "BSplineSmoother::smooth() requires >= 2 waypoints, got " +
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

  // ==============================
  // Gradient Descent Optimization
  // ==============================
  std::vector<std::pair<double, double>> smoothed(waypoints);

  for (int iter = 0; iter < max_iterations_; ++iter) {
    double change = 0.0;

    for (size_t i = 1; i < n - 1; ++i) {
      double old_x = smoothed[i].first;
      double old_y = smoothed[i].second;

      // Data fidelity term: pull toward original waypoints
      smoothed[i].first += weight_data_ * (waypoints[i].first - smoothed[i].first);
      smoothed[i].second += weight_data_ * (waypoints[i].second - smoothed[i].second);

      // Smoothness term: minimize second derivative
      smoothed[i].first += weight_smooth_ *
        (smoothed[i - 1].first + smoothed[i + 1].first - 2.0 * smoothed[i].first);
      smoothed[i].second += weight_smooth_ *
        (smoothed[i - 1].second + smoothed[i + 1].second - 2.0 * smoothed[i].second);

      change += std::abs(smoothed[i].first - old_x) +
                std::abs(smoothed[i].second - old_y);
    }

    if (change < tolerance_) {
      break;  // Converged
    }
  }

  // ==============================
  // Upsample to num_points via linear interpolation on arc-length
  // ==============================
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    double dx = smoothed[i].first - smoothed[i - 1].first;
    double dy = smoothed[i].second - smoothed[i - 1].second;
    arc[i] = arc[i - 1] + std::hypot(dx, dy);
  }
  double total_len = arc.back();

  std::vector<PathPoint> result;
  result.reserve(num_points);

  for (int k = 0; k < num_points; ++k) {
    double s = total_len * static_cast<double>(k) / (num_points - 1);

    // Find segment
    size_t seg = 0;
    for (size_t i = 0; i < n - 1; ++i) {
      if (s <= arc[i + 1]) {
        seg = i;
        break;
      }
      seg = i;
    }

    double seg_len = arc[seg + 1] - arc[seg];
    double t = (seg_len > 1e-12) ? (s - arc[seg]) / seg_len : 0.0;

    double x = smoothed[seg].first + t * (smoothed[seg + 1].first - smoothed[seg].first);
    double y = smoothed[seg].second + t * (smoothed[seg + 1].second - smoothed[seg].second);

    double theta = 0.0;
    if (seg + 1 < n) {
      theta = std::atan2(smoothed[seg + 1].second - smoothed[seg].second,
                         smoothed[seg + 1].first - smoothed[seg].first);
    } else if (!result.empty()) {
      theta = result.back().theta;
    }

    PathPoint pt;
    pt.x = x;
    pt.y = y;
    pt.theta = theta;
    pt.arc_length = s;
    pt.curvature = 0.0;
    result.push_back(pt);
  }

  // Compute curvature at each interior point
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
