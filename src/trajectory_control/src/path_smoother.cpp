/**
 * @file path_smoother.cpp
 * @brief Implementation of path smoothing algorithms.
 *
 * Contains:
 *   1. Cubic Spline Interpolation — solves tridiagonal system for natural spline
 *   2. Gradient Descent Smoothing — iterative path optimization
 */

#include "trajectory_control/path_smoother.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace trajectory_control
{

PathSmoother::PathSmoother() = default;

void PathSmoother::setMethod(SmoothingMethod method)
{
  method_ = method;
}

void PathSmoother::setNumPoints(int num_points)
{
  if (num_points < 2) {
    throw std::invalid_argument("Number of output points must be >= 2");
  }
  num_points_ = num_points;
}

void PathSmoother::setGradientDescentParams(
  double weight_data, double weight_smooth, double tolerance, int max_iterations)
{
  weight_data_ = weight_data;
  weight_smooth_ = weight_smooth;
  tolerance_ = tolerance;
  max_iterations_ = max_iterations;
}

std::vector<Pose2D> PathSmoother::smooth(
  const std::vector<std::pair<double, double>> & waypoints) const
{
  // --- Input validation ---
  if (waypoints.size() < 2) {
    throw std::invalid_argument(
      "PathSmoother::smooth() requires at least 2 waypoints, got " +
      std::to_string(waypoints.size()));
  }

  // Check for NaN/Inf
  for (const auto & wp : waypoints) {
    if (std::isnan(wp.first) || std::isnan(wp.second) ||
        std::isinf(wp.first) || std::isinf(wp.second))
    {
      throw std::invalid_argument("Waypoints contain NaN or Inf values");
    }
  }

  switch (method_) {
    case SmoothingMethod::CUBIC_SPLINE:
      return cubicSplineSmooth(waypoints);
    case SmoothingMethod::GRADIENT_DESCENT:
      return gradientDescentSmooth(waypoints);
    default:
      return cubicSplineSmooth(waypoints);
  }
}

// ==========================================================================
// Cubic Spline Interpolation
// ==========================================================================

std::vector<Pose2D> PathSmoother::cubicSplineSmooth(
  const std::vector<std::pair<double, double>> & waypoints) const
{
  const size_t n = waypoints.size();

  // Special case: exactly 2 points → linear interpolation
  if (n == 2) {
    std::vector<Pose2D> result;
    result.reserve(num_points_);
    double theta = std::atan2(waypoints[1].second - waypoints[0].second,
                              waypoints[1].first - waypoints[0].first);
    for (int i = 0; i < num_points_; ++i) {
      double t = static_cast<double>(i) / (num_points_ - 1);
      double x = waypoints[0].first + t * (waypoints[1].first - waypoints[0].first);
      double y = waypoints[0].second + t * (waypoints[1].second - waypoints[0].second);
      result.emplace_back(x, y, theta);
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
  std::vector<double> ax, bx, cx, dx;
  std::vector<double> ay, by, cy, dy;
  solveNaturalCubicSpline(arc, wx, ax, bx, cx, dx);
  solveNaturalCubicSpline(arc, wy, ay, by, cy, dy);

  // Step 4: Sample the spline at uniform arc-length intervals
  double total_length = arc.back();
  std::vector<Pose2D> result;
  result.reserve(num_points_);

  for (int i = 0; i < num_points_; ++i) {
    double s = total_length * static_cast<double>(i) / (num_points_ - 1);
    double x = evaluateSpline(s, arc, ax, bx, cx, dx);
    double y = evaluateSpline(s, arc, ay, by, cy, dy);

    // Compute heading from path tangent (derivative of spline)
    double theta = 0.0;
    if (i < num_points_ - 1) {
      double s_next = total_length * static_cast<double>(i + 1) / (num_points_ - 1);
      double x_next = evaluateSpline(s_next, arc, ax, bx, cx, dx);
      double y_next = evaluateSpline(s_next, arc, ay, by, cy, dy);
      theta = std::atan2(y_next - y, x_next - x);
    } else if (result.size() > 0) {
      theta = result.back().theta;  // Last point: use previous heading
    }

    result.emplace_back(x, y, theta);
  }

  return result;
}

void PathSmoother::solveNaturalCubicSpline(
  const std::vector<double> & t,
  const std::vector<double> & y,
  std::vector<double> & a,
  std::vector<double> & b,
  std::vector<double> & c,
  std::vector<double> & d) const
{
  /**
   * Natural Cubic Spline: S''(t_0) = 0, S''(t_n) = 0
   *
   * For each segment i: S_i(x) = a_i + b_i*(x-t_i) + c_i*(x-t_i)^2 + d_i*(x-t_i)^3
   *
   * Solve tridiagonal system for c[] coefficients, then back-substitute
   * for b[] and d[].
   */
  const int n = static_cast<int>(t.size()) - 1;  // number of segments

  a.resize(n + 1);
  b.resize(n);
  c.resize(n + 1);
  d.resize(n);

  // a_i = y_i
  for (int i = 0; i <= n; ++i) {
    a[i] = y[i];
  }

  // h_i = t_{i+1} - t_i
  std::vector<double> h(n);
  for (int i = 0; i < n; ++i) {
    h[i] = t[i + 1] - t[i];
    if (h[i] < 1e-12) {
      h[i] = 1e-12;  // Prevent division by zero for duplicate points
    }
  }

  // Set up tridiagonal system for c[]
  // alpha_i = 3/h_i * (a_{i+1} - a_i) - 3/h_{i-1} * (a_i - a_{i-1})
  std::vector<double> alpha(n + 1, 0.0);
  for (int i = 1; i < n; ++i) {
    alpha[i] = 3.0 / h[i] * (a[i + 1] - a[i]) -
               3.0 / h[i - 1] * (a[i] - a[i - 1]);
  }

  // Solve tridiagonal system using Thomas algorithm
  std::vector<double> l(n + 1, 1.0);
  std::vector<double> mu(n + 1, 0.0);
  std::vector<double> z(n + 1, 0.0);

  for (int i = 1; i < n; ++i) {
    l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
    if (std::abs(l[i]) < 1e-12) l[i] = 1e-12;
    mu[i] = h[i] / l[i];
    z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
  }

  // Back substitution
  c[n] = 0.0;  // Natural spline boundary condition
  for (int j = n - 1; j >= 0; --j) {
    c[j] = z[j] - mu[j] * c[j + 1];
    b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
    d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
  }
}

double PathSmoother::evaluateSpline(
  double t_eval,
  const std::vector<double> & t,
  const std::vector<double> & a,
  const std::vector<double> & b,
  const std::vector<double> & c,
  const std::vector<double> & d) const
{
  // Find the segment index
  const int n = static_cast<int>(t.size()) - 1;
  int seg = n - 1;  // default to last segment

  for (int i = 0; i < n; ++i) {
    if (t_eval <= t[i + 1]) {
      seg = i;
      break;
    }
  }

  // Clamp to valid range
  seg = std::max(0, std::min(seg, n - 1));

  double dt = t_eval - t[seg];
  return a[seg] + b[seg] * dt + c[seg] * dt * dt + d[seg] * dt * dt * dt;
}

std::vector<double> PathSmoother::computeArcLengths(
  const std::vector<std::pair<double, double>> & waypoints) const
{
  std::vector<double> arc(waypoints.size(), 0.0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    double dx = waypoints[i].first - waypoints[i - 1].first;
    double dy = waypoints[i].second - waypoints[i - 1].second;
    arc[i] = arc[i - 1] + std::hypot(dx, dy);
  }
  return arc;
}

// ==========================================================================
// Gradient Descent Smoothing
// ==========================================================================

std::vector<Pose2D> PathSmoother::gradientDescentSmooth(
  const std::vector<std::pair<double, double>> & waypoints) const
{
  /**
   * Gradient Descent Path Smoothing (from Udacity/Sebastian Thrun):
   *
   * Minimize: F = alpha * ||new - original||^2 + beta * ||new_i - new_{i-1}||^2
   *
   * Update rule:
   *   new[i] += alpha * (original[i] - new[i])
   *   new[i] += beta  * (new[i-1] + new[i+1] - 2*new[i])
   */
  const size_t n = waypoints.size();

  // Initialize smoothed path as copy of original
  std::vector<std::pair<double, double>> smoothed(waypoints);

  for (int iter = 0; iter < max_iterations_; ++iter) {
    double change = 0.0;

    for (size_t i = 1; i < n - 1; ++i) {
      double old_x = smoothed[i].first;
      double old_y = smoothed[i].second;

      // Data fidelity: pull toward original
      smoothed[i].first += weight_data_ * (waypoints[i].first - smoothed[i].first);
      smoothed[i].second += weight_data_ * (waypoints[i].second - smoothed[i].second);

      // Smoothness: minimize second derivative
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

  // Upsample to num_points_ by linear interpolation
  std::vector<Pose2D> result;
  result.reserve(num_points_);

  // Compute arc lengths of smoothed path
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    double dx = smoothed[i].first - smoothed[i - 1].first;
    double dy = smoothed[i].second - smoothed[i - 1].second;
    arc[i] = arc[i - 1] + std::hypot(dx, dy);
  }
  double total_len = arc.back();

  for (int k = 0; k < num_points_; ++k) {
    double s = total_len * static_cast<double>(k) / (num_points_ - 1);

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

    result.emplace_back(x, y, theta);
  }

  return result;
}

}  // namespace trajectory_control
