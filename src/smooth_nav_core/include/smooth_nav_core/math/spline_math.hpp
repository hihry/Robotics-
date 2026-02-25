/**
 * @file spline_math.hpp
 * @brief Tridiagonal solver / Thomas algorithm for cubic spline interpolation.
 *
 * Pure C++ — no ROS dependencies.
 */

#ifndef SMOOTH_NAV_CORE__MATH__SPLINE_MATH_HPP_
#define SMOOTH_NAV_CORE__MATH__SPLINE_MATH_HPP_

#include <vector>
#include <cmath>
#include <algorithm>

namespace smooth_nav_core
{

/**
 * @brief Coefficients for a piecewise cubic polynomial.
 *
 * For segment i: S_i(t) = a[i] + b[i]*(t-t_i) + c[i]*(t-t_i)^2 + d[i]*(t-t_i)^3
 */
struct SplineCoefficients
{
  std::vector<double> a;  ///< constant coefficients
  std::vector<double> b;  ///< linear coefficients
  std::vector<double> c;  ///< quadratic coefficients
  std::vector<double> d;  ///< cubic coefficients
};

/**
 * @brief Solve natural cubic spline for 1D data using Thomas algorithm.
 *
 * Natural boundary conditions: S''(t_0) = 0, S''(t_n) = 0
 *
 * @param t Parameter values (e.g., arc lengths), size n+1
 * @param y Data values, size n+1
 * @return SplineCoefficients for n segments
 */
inline SplineCoefficients solveNaturalCubicSpline(
  const std::vector<double> & t,
  const std::vector<double> & y)
{
  const int n = static_cast<int>(t.size()) - 1;
  SplineCoefficients coeff;
  coeff.a.resize(n + 1);
  coeff.b.resize(n);
  coeff.c.resize(n + 1);
  coeff.d.resize(n);

  for (int i = 0; i <= n; ++i) {
    coeff.a[i] = y[i];
  }

  std::vector<double> h(n);
  for (int i = 0; i < n; ++i) {
    h[i] = t[i + 1] - t[i];
    if (h[i] < 1e-12) h[i] = 1e-12;
  }

  std::vector<double> alpha(n + 1, 0.0);
  for (int i = 1; i < n; ++i) {
    alpha[i] = 3.0 / h[i] * (coeff.a[i + 1] - coeff.a[i]) -
               3.0 / h[i - 1] * (coeff.a[i] - coeff.a[i - 1]);
  }

  std::vector<double> l(n + 1, 1.0);
  std::vector<double> mu(n + 1, 0.0);
  std::vector<double> z(n + 1, 0.0);

  for (int i = 1; i < n; ++i) {
    l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
    if (std::abs(l[i]) < 1e-12) l[i] = 1e-12;
    mu[i] = h[i] / l[i];
    z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
  }

  coeff.c[n] = 0.0;
  for (int j = n - 1; j >= 0; --j) {
    coeff.c[j] = z[j] - mu[j] * coeff.c[j + 1];
    coeff.b[j] = (coeff.a[j + 1] - coeff.a[j]) / h[j] -
                  h[j] * (coeff.c[j + 1] + 2.0 * coeff.c[j]) / 3.0;
    coeff.d[j] = (coeff.c[j + 1] - coeff.c[j]) / (3.0 * h[j]);
  }

  return coeff;
}

/**
 * @brief Evaluate a cubic spline at a given parameter value.
 */
inline double evaluateSpline(
  double t_eval,
  const std::vector<double> & t,
  const SplineCoefficients & coeff)
{
  const int n = static_cast<int>(t.size()) - 1;
  int seg = n - 1;

  for (int i = 0; i < n; ++i) {
    if (t_eval <= t[i + 1]) {
      seg = i;
      break;
    }
  }
  seg = std::max(0, std::min(seg, n - 1));

  double dt = t_eval - t[seg];
  return coeff.a[seg] + coeff.b[seg] * dt +
         coeff.c[seg] * dt * dt + coeff.d[seg] * dt * dt * dt;
}

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__MATH__SPLINE_MATH_HPP_
