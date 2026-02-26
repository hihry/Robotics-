/**
 * @file spline_math.hpp
 * @brief Natural cubic spline solver & evaluator — pure C++17, no ROS.
 *
 * Solves the tridiagonal system arising from natural boundary conditions
 * (S''(t₀) = 0, S''(tₙ) = 0) using the Thomas algorithm — O(n) time, O(n) space.
 *
 * Provides exact analytic evaluation of:
 *   • S(t)   — position
 *   • S'(t)  — first derivative (tangent)
 *   • S''(t) — second derivative (used in curvature computation)
 *
 * Reference:
 *   Burden & Faires, *Numerical Analysis*, 10th ed., §3.5
 */

#ifndef SMOOTH_NAV_CORE__MATH__SPLINE_MATH_HPP_
#define SMOOTH_NAV_CORE__MATH__SPLINE_MATH_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace smooth_nav_core
{

/**
 * @brief Coefficients for a piecewise cubic polynomial.
 *
 * For segment i:
 *   S_i(t) = a[i] + b[i]·(t-tᵢ) + c[i]·(t-tᵢ)² + d[i]·(t-tᵢ)³
 *
 * Size convention:
 *   a — n+1 entries (values at each knot)
 *   b, c — n+1 entries (c[n] = 0 for natural BC)
 *   d — n   entries (one per segment)
 */
struct SplineCoefficients
{
  std::vector<double> a;  ///< constant   (position at knot i)
  std::vector<double> b;  ///< linear     (first derivative at knot i)
  std::vector<double> c;  ///< quadratic  (half of second derivative at knot i)
  std::vector<double> d;  ///< cubic
};

// ─────────────────────────────────────────────────────────────────────────────
// Solver
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Solve natural cubic spline for 1-D data using the Thomas algorithm.
 *
 * Natural boundary conditions: S''(t₀) = 0, S''(tₙ) = 0.
 *
 * @param t Parameter values (e.g., cumulative arc lengths). Size n+1, strictly increasing.
 * @param y Data values at each knot.  Size n+1.
 * @return SplineCoefficients with n segments.
 * @throws std::invalid_argument if sizes mismatch or n < 1.
 */
inline SplineCoefficients solveNaturalCubicSpline(
  const std::vector<double> & t,
  const std::vector<double> & y)
{
  if (t.size() != y.size() || t.size() < 2) {
    throw std::invalid_argument(
      "solveNaturalCubicSpline: t and y must have >= 2 elements and equal size.");
  }

  const int n = static_cast<int>(t.size()) - 1;
  SplineCoefficients coeff;
  coeff.a.resize(n + 1);
  coeff.b.resize(n + 1);          // store b at every knot (useful for derivative)
  coeff.c.resize(n + 1);
  coeff.d.resize(n);

  // Copy data values
  for (int i = 0; i <= n; ++i) {
    coeff.a[i] = y[i];
  }

  // Step widths
  std::vector<double> h(n);
  for (int i = 0; i < n; ++i) {
    h[i] = t[i + 1] - t[i];
    if (h[i] < 1e-15) h[i] = 1e-15;  // guard against zero-length segments
  }

  // Right-hand side of tridiagonal system
  std::vector<double> alpha(n + 1, 0.0);
  for (int i = 1; i < n; ++i) {
    alpha[i] = 3.0 / h[i] * (coeff.a[i + 1] - coeff.a[i]) -
               3.0 / h[i - 1] * (coeff.a[i] - coeff.a[i - 1]);
  }

  // Thomas algorithm — forward sweep
  std::vector<double> l(n + 1, 1.0);
  std::vector<double> mu(n + 1, 0.0);
  std::vector<double> z(n + 1, 0.0);

  for (int i = 1; i < n; ++i) {
    l[i]  = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
    if (std::abs(l[i]) < 1e-15) l[i] = 1e-15;
    mu[i] = h[i] / l[i];
    z[i]  = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
  }

  // Back-substitution
  coeff.c[n] = 0.0;                  // natural BC: S''(tₙ) = 0
  for (int j = n - 1; j >= 0; --j) {
    coeff.c[j] = z[j] - mu[j] * coeff.c[j + 1];
    coeff.b[j] = (coeff.a[j + 1] - coeff.a[j]) / h[j] -
                  h[j] * (coeff.c[j + 1] + 2.0 * coeff.c[j]) / 3.0;
    coeff.d[j] = (coeff.c[j + 1] - coeff.c[j]) / (3.0 * h[j]);
  }
  // b at last knot (extrapolation convenience)
  coeff.b[n] = coeff.b[n - 1] + 2.0 * coeff.c[n - 1] * h[n - 1] +
               3.0 * coeff.d[n - 1] * h[n - 1] * h[n - 1];

  return coeff;
}

// ─────────────────────────────────────────────────────────────────────────────
// Segment finder (shared helper)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Find the segment index for a given parameter value using binary search.
 *
 * Returns index i such that t[i] ≤ t_eval < t[i+1], clamped to [0, n-1].
 */
inline int findSegment(double t_eval, const std::vector<double> & t)
{
  const int n = static_cast<int>(t.size()) - 1;
  if (t_eval <= t[0])   return 0;
  if (t_eval >= t[n])   return n - 1;

  // Binary search for the interval
  int lo = 0, hi = n;
  while (lo < hi - 1) {
    int mid = (lo + hi) / 2;
    if (t_eval < t[mid]) hi = mid;
    else                 lo = mid;
  }
  return lo;
}

// ─────────────────────────────────────────────────────────────────────────────
// Evaluators
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Evaluate S(t_eval) — spline position.
 */
inline double evaluateSpline(
  double t_eval,
  const std::vector<double> & t,
  const SplineCoefficients & coeff)
{
  int seg = findSegment(t_eval, t);
  double dt = t_eval - t[seg];
  return coeff.a[seg] + coeff.b[seg] * dt +
         coeff.c[seg] * dt * dt + coeff.d[seg] * dt * dt * dt;
}

/**
 * @brief Evaluate S'(t_eval) — first derivative (tangent).
 *
 * S'(t) = b + 2c·Δt + 3d·Δt²
 */
inline double evaluateSplineDerivative(
  double t_eval,
  const std::vector<double> & t,
  const SplineCoefficients & coeff)
{
  int seg = findSegment(t_eval, t);
  double dt = t_eval - t[seg];
  return coeff.b[seg] + 2.0 * coeff.c[seg] * dt +
         3.0 * coeff.d[seg] * dt * dt;
}

/**
 * @brief Evaluate S''(t_eval) — second derivative.
 *
 * S''(t) = 2c + 6d·Δt
 */
inline double evaluateSplineSecondDerivative(
  double t_eval,
  const std::vector<double> & t,
  const SplineCoefficients & coeff)
{
  int seg = findSegment(t_eval, t);
  double dt = t_eval - t[seg];
  return 2.0 * coeff.c[seg] + 6.0 * coeff.d[seg] * dt;
}

/**
 * @brief Compute exact signed curvature from two parametric splines x(s), y(s).
 *
 *   κ(s) = (x'·y'' − y'·x'') / (x'² + y'²)^(3/2)
 *
 * Uses analytic derivatives — no finite-difference approximation.
 */
inline double computeExactCurvature(
  double s,
  const std::vector<double> & knots,
  const SplineCoefficients & cx,
  const SplineCoefficients & cy)
{
  double dx  = evaluateSplineDerivative(s, knots, cx);
  double dy  = evaluateSplineDerivative(s, knots, cy);
  double ddx = evaluateSplineSecondDerivative(s, knots, cx);
  double ddy = evaluateSplineSecondDerivative(s, knots, cy);

  double speed_sq = dx * dx + dy * dy;
  double denom = std::pow(speed_sq, 1.5);
  if (denom < 1e-15) return 0.0;
  return (dx * ddy - dy * ddx) / denom;
}

/**
 * @brief Compute exact heading (tangent angle) from two parametric splines.
 *
 *   θ(s) = atan2(y'(s), x'(s))
 */
inline double computeExactHeading(
  double s,
  const std::vector<double> & knots,
  const SplineCoefficients & cx,
  const SplineCoefficients & cy)
{
  double dx = evaluateSplineDerivative(s, knots, cx);
  double dy = evaluateSplineDerivative(s, knots, cy);
  return std::atan2(dy, dx);
}

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__MATH__SPLINE_MATH_HPP_
