/**
 * @file trajectory_utils.hpp
 * @brief Utility functions for trajectory generation — arc-length, curvature, interpolation.
 */

#ifndef SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAJECTORY_UTILS_HPP_
#define SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAJECTORY_UTILS_HPP_

#include "smooth_nav_core/math/types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace smooth_nav_core
{

/**
 * @brief Compute cumulative arc lengths for a PathPoint sequence.
 */
inline std::vector<double> computePathArcLengths(const std::vector<PathPoint> & path)
{
  std::vector<double> arc(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] + std::hypot(path[i].x - path[i - 1].x,
                                       path[i].y - path[i - 1].y);
  }
  return arc;
}

/**
 * @brief Compute total arc length of a path.
 */
inline double computeTotalArcLength(const std::vector<PathPoint> & path)
{
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return total;
}

/**
 * @brief Compute curvature at a point using central finite differences.
 *
 * κ = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
 */
inline double computeCurvature(const std::vector<PathPoint> & path, size_t index)
{
  if (path.size() < 3 || index == 0 || index >= path.size() - 1) {
    return 0.0;
  }
  const auto & prev = path[index - 1];
  const auto & curr = path[index];
  const auto & next = path[index + 1];

  double dx = (next.x - prev.x) / 2.0;
  double dy = (next.y - prev.y) / 2.0;
  double ddx = next.x - 2.0 * curr.x + prev.x;
  double ddy = next.y - 2.0 * curr.y + prev.y;

  double denom = std::pow(dx * dx + dy * dy, 1.5);
  if (denom < 1e-12) return 0.0;

  return (dx * ddy - dy * ddx) / denom;
}

/**
 * @brief Interpolate position on path at a given arc length.
 */
inline PathPoint interpolateAtArcLength(
  const std::vector<PathPoint> & path,
  const std::vector<double> & arc_lengths,
  double target_s)
{
  target_s = std::max(0.0, std::min(target_s, arc_lengths.back()));

  size_t seg = 0;
  for (size_t i = 0; i < arc_lengths.size() - 1; ++i) {
    if (target_s <= arc_lengths[i + 1]) {
      seg = i;
      break;
    }
    seg = i;
  }

  double seg_len = arc_lengths[seg + 1] - arc_lengths[seg];
  double t = (seg_len > 1e-12) ? (target_s - arc_lengths[seg]) / seg_len : 0.0;
  t = std::max(0.0, std::min(t, 1.0));

  PathPoint result;
  result.x = path[seg].x + t * (path[seg + 1].x - path[seg].x);
  result.y = path[seg].y + t * (path[seg + 1].y - path[seg].y);
  result.theta = std::atan2(path[seg + 1].y - path[seg].y,
                             path[seg + 1].x - path[seg].x);
  result.arc_length = target_s;
  return result;
}

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAJECTORY_UTILS_HPP_
