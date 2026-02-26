/**
 * @file bspline_smoother.cpp
 * @brief Gradient-descent path smoother with optional obstacle avoidance.
 *
 * Cost function:
 *   F = α Σ‖pᵢ − qᵢ‖²  +  β Σ‖pᵢ₋₁ − 2pᵢ + pᵢ₊₁‖²  +  γ Σ obstacle_repulsion
 *       (data fidelity)       (smoothness / curvature)       (obstacle avoidance)
 *
 * After convergence, the optimised points are up-sampled with a natural
 * cubic spline (via spline_math.hpp) for C²-continuous output.
 *
 * References:
 *   • Sebastian Thrun, "CS373 — Programming a Robotic Car" (Udacity, 2012)
 *   • Ziegler & Stiller, "Spatiotemporal state lattices for fast trajectory
 *     planning in dynamic on-road driving scenarios", IROS 2009
 */

#include "smooth_nav_core/path_smoother/bspline_smoother.hpp"
#include "smooth_nav_core/math/geometry_utils.hpp"
#include "smooth_nav_core/math/spline_math.hpp"
#include <cmath>
#include <stdexcept>

namespace smooth_nav_core
{

// ─── Configuration helpers ──────────────────────────────────────────────────

void BSplineSmoother::setParams(
  double weight_data, double weight_smooth,
  double tolerance, int max_iterations)
{
  weight_data_    = weight_data;
  weight_smooth_  = weight_smooth;
  tolerance_      = tolerance;
  max_iterations_ = max_iterations;
}

void BSplineSmoother::setObstacles(
  const std::vector<Obstacle> & obstacles,
  double weight_obstacle,
  double safety_margin)
{
  obstacles_       = obstacles;
  weight_obstacle_ = weight_obstacle;
  safety_margin_   = safety_margin;
}

// ─── Main algorithm ─────────────────────────────────────────────────────────

std::vector<PathPoint> BSplineSmoother::smooth(
  const std::vector<std::pair<double, double>> & raw_waypoints,
  int num_points) const
{
  // ────────────────── Validation ──────────────────
  if (raw_waypoints.size() < 2) {
    throw std::invalid_argument(
      "BSplineSmoother::smooth() requires >= 2 waypoints, got " +
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

  // Remove consecutive duplicate points
  auto waypoints = removeDuplicates(raw_waypoints);
  if (waypoints.size() < 2) {
    throw std::invalid_argument(
      "After duplicate removal, fewer than 2 unique waypoints remain");
  }

  const size_t n = waypoints.size();

  // ════════════════════════════════════════════════
  //  Phase 1 — Gradient Descent Optimisation
  // ════════════════════════════════════════════════
  std::vector<std::pair<double, double>> smoothed(waypoints);

  for (int iter = 0; iter < max_iterations_; ++iter) {
    double change = 0.0;

    for (size_t i = 1; i < n - 1; ++i) {
      double old_x = smoothed[i].first;
      double old_y = smoothed[i].second;

      // ── Data fidelity: pull toward original ──
      smoothed[i].first  += weight_data_ * (waypoints[i].first  - smoothed[i].first);
      smoothed[i].second += weight_data_ * (waypoints[i].second - smoothed[i].second);

      // ── Smoothness: minimise second difference ──
      smoothed[i].first  += weight_smooth_ *
        (smoothed[i - 1].first + smoothed[i + 1].first - 2.0 * smoothed[i].first);
      smoothed[i].second += weight_smooth_ *
        (smoothed[i - 1].second + smoothed[i + 1].second - 2.0 * smoothed[i].second);

      // ── Obstacle repulsion (potential field) ──
      if (weight_obstacle_ > 0.0 && !obstacles_.empty()) {
        for (const auto & obs : obstacles_) {
          double dx = smoothed[i].first  - obs.x;
          double dy = smoothed[i].second - obs.y;
          double d  = std::hypot(dx, dy);
          double clearance = d - obs.radius;

          if (clearance < safety_margin_ && d > 1e-9) {
            // Repulsive gradient: magnitude ∝ 1/(clearance)² , direction = away
            double repulsion = weight_obstacle_ / std::max(clearance * clearance, 1e-6);
            smoothed[i].first  += repulsion * (dx / d);
            smoothed[i].second += repulsion * (dy / d);
          }
        }
      }

      change += std::abs(smoothed[i].first  - old_x) +
                std::abs(smoothed[i].second - old_y);
    }

    if (change < tolerance_) break;   // converged
  }

  // ════════════════════════════════════════════════
  //  Phase 2 — Cubic-spline up-sampling (C² output)
  // ════════════════════════════════════════════════
  // Compute arc-length parameter from optimised control points
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    arc[i] = arc[i - 1] + std::hypot(smoothed[i].first  - smoothed[i - 1].first,
                                      smoothed[i].second - smoothed[i - 1].second);
  }
  double total_len = arc.back();

  if (total_len < 1e-12) {
    // Degenerate: all points collapsed to the same location → flat output
    std::vector<PathPoint> result(num_points);
    for (auto & pt : result) { pt.x = smoothed[0].first; pt.y = smoothed[0].second; }
    return result;
  }

  // Build x(s), y(s) splines over the optimised control points
  std::vector<double> sx(n), sy(n);
  for (size_t i = 0; i < n; ++i) {
    sx[i] = smoothed[i].first;
    sy[i] = smoothed[i].second;
  }

  SplineCoefficients cx = solveNaturalCubicSpline(arc, sx);
  SplineCoefficients cy = solveNaturalCubicSpline(arc, sy);

  // Sample at uniform arc-length intervals
  std::vector<PathPoint> result;
  result.reserve(num_points);

  for (int k = 0; k < num_points; ++k) {
    double s = total_len * static_cast<double>(k) / (num_points - 1);

    PathPoint pt;
    pt.x         = evaluateSpline(s, arc, cx);
    pt.y         = evaluateSpline(s, arc, cy);
    pt.theta     = computeExactHeading(s, arc, cx, cy);
    pt.curvature = computeExactCurvature(s, arc, cx, cy);
    pt.arc_length = s;
    result.push_back(pt);
  }

  // Pin endpoints exactly
  result.front().x = smoothed.front().first;
  result.front().y = smoothed.front().second;
  result.back().x  = smoothed.back().first;
  result.back().y  = smoothed.back().second;

  return result;
}

}  // namespace smooth_nav_core
