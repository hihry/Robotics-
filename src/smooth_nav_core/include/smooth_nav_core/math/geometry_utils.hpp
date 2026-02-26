/**
 * @file geometry_utils.hpp
 * @brief Geometry utility functions — pure C++17, no ROS.
 *
 * All functions are header-only (inline) for zero link-time overhead.
 */

#ifndef SMOOTH_NAV_CORE__MATH__GEOMETRY_UTILS_HPP_
#define SMOOTH_NAV_CORE__MATH__GEOMETRY_UTILS_HPP_

// Ensure M_PI is defined on MSVC
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <numeric>

// Portable M_PI fallback
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace smooth_nav_core
{

// ─── Scalar helpers ──────────────────────────────────────────────────────────

/**
 * @brief Euclidean distance between two 2-D points.
 */
inline double dist(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

/**
 * @brief Normalize angle to (-π, π] using std::remainder (branch-free, O(1)).
 *
 * This is strictly superior to the naive while-loop approach which can loop
 * indefinitely for large angles.
 */
inline double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

/**
 * @brief Shortest signed angular difference from `a` to `b`.
 *
 * Positive = counter-clockwise.  Result in (-π, π].
 */
inline double angleDiff(double a, double b)
{
  return normalizeAngle(b - a);
}

/**
 * @brief Linear interpolation between two scalars.
 */
inline double lerp(double a, double b, double t)
{
  return a + t * (b - a);
}

/**
 * @brief Clamp a value to [lo, hi].
 */
inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

// ─── 2-D vector algebra ─────────────────────────────────────────────────────

/** @brief 2-D dot product: a · b */
inline double dot2D(double ax, double ay, double bx, double by)
{
  return ax * bx + ay * by;
}

/** @brief 2-D cross product (z-component): a × b */
inline double cross2D(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

// ─── Segment / point geometry ────────────────────────────────────────────────

/**
 * @brief Closest point on segment AB to point P.
 *
 * @param[out] t  Parametric position on segment [0,1].
 * @return (cx, cy) closest point on AB.
 */
inline std::pair<double, double> closestPointOnSegment(
  double ax, double ay, double bx, double by,
  double px, double py, double & t)
{
  double dx = bx - ax, dy = by - ay;
  double len_sq = dx * dx + dy * dy;
  if (len_sq < 1e-24) {
    t = 0.0;
    return {ax, ay};
  }
  t = clamp(((px - ax) * dx + (py - ay) * dy) / len_sq, 0.0, 1.0);
  return {ax + t * dx, ay + t * dy};
}

/**
 * @brief Signed distance from point P to line through A→B.
 *
 * Positive = P is to the *left* of AB (CCW sense).
 */
inline double signedDistToLine(
  double ax, double ay, double bx, double by,
  double px, double py)
{
  double len = std::hypot(bx - ax, by - ay);
  if (len < 1e-12) return std::hypot(px - ax, py - ay);
  return cross2D(bx - ax, by - ay, px - ax, py - ay) / len;
}

/**
 * @brief Unsigned distance from point P to segment AB.
 */
inline double pointToSegmentDistance(
  double ax, double ay, double bx, double by,
  double px, double py)
{
  double t;
  auto [cx, cy] = closestPointOnSegment(ax, ay, bx, by, px, py, t);
  return std::hypot(px - cx, py - cy);
}

// ─── Arc-length utilities ────────────────────────────────────────────────────

/**
 * @brief Compute cumulative arc lengths from a sequence of (x,y) pairs.
 */
inline std::vector<double> computeArcLengths(
  const std::vector<std::pair<double, double>> & points)
{
  std::vector<double> arc(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    double dx = points[i].first  - points[i - 1].first;
    double dy = points[i].second - points[i - 1].second;
    arc[i] = arc[i - 1] + std::hypot(dx, dy);
  }
  return arc;
}

/**
 * @brief Remove consecutive duplicate points (dist < eps) from a waypoint list.
 *
 * Degenerate zero-length segments cause division-by-zero in spline solvers.
 */
inline std::vector<std::pair<double, double>> removeDuplicates(
  const std::vector<std::pair<double, double>> & pts, double eps = 1e-9)
{
  if (pts.empty()) return {};
  std::vector<std::pair<double, double>> out;
  out.reserve(pts.size());
  out.push_back(pts[0]);
  for (size_t i = 1; i < pts.size(); ++i) {
    double d = std::hypot(pts[i].first - out.back().first,
                          pts[i].second - out.back().second);
    if (d > eps) {
      out.push_back(pts[i]);
    }
  }
  return out;
}

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__MATH__GEOMETRY_UTILS_HPP_
