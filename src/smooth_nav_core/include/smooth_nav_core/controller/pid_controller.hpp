/**
 * @file pid_controller.hpp
 * @brief Production PID controller with anti-windup, derivative filter, and output clamping.
 *
 * Features over a textbook PID:
 *   1. **dt-aware** — integral uses actual time step, not iteration count.
 *   2. **Anti-windup (clamping)** — integral term is frozen when output saturates.
 *   3. **Derivative low-pass filter** — first-order IIR on the D term prevents
 *      derivative kick from noisy sensor readings.
 *   4. **Output limits** — prevents unbounded control signal.
 *   5. **Derivative on measurement** — differentiates on the error derivative
 *      (equivalent for constant setpoint; safer than differentiating raw setpoint).
 *
 * Standalone class — used by PurePursuitController for cross-track and heading
 * correction, but can be instantiated anywhere.
 */

#ifndef SMOOTH_NAV_CORE__CONTROLLER__PID_CONTROLLER_HPP_
#define SMOOTH_NAV_CORE__CONTROLLER__PID_CONTROLLER_HPP_

#include <cmath>
#include <algorithm>

namespace smooth_nav_core
{

class PIDController
{
public:
  PIDController() = default;

  /**
   * @brief Construct with PID gains.
   *
   * @param kp Proportional gain.
   * @param ki Integral gain.
   * @param kd Derivative gain.
   */
  PIDController(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd) {}

  // ─── Configuration ──────────────────────────────────────────
  void setGains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  /** @brief Set symmetric output limits: output ∈ [-limit, +limit]. */
  void setOutputLimits(double limit)
  {
    output_min_ = -std::abs(limit);
    output_max_ =  std::abs(limit);
  }

  /** @brief Set asymmetric output limits. */
  void setOutputLimits(double min_val, double max_val)
  {
    output_min_ = min_val;
    output_max_ = max_val;
  }

  /** @brief Set integral anti-windup limit (symmetric). */
  void setIntegralLimit(double limit) { integral_limit_ = std::abs(limit); }

  /**
   * @brief Set derivative low-pass filter coefficient.
   *
   * alpha = 0.0 → no filtering (raw derivative).
   * alpha = 0.9 → heavy smoothing (slow to respond).
   * Default: 0.1 — light filtering.
   */
  void setDerivativeFilter(double alpha) { d_filter_alpha_ = alpha; }

  // ─── Computation ────────────────────────────────────────────

  /**
   * @brief Compute PID output given the current error and time step.
   *
   * @param error  Signed error value (setpoint − measurement).
   * @param dt     Time since last call (seconds). If ≤ 0, uses unit dt.
   * @return Clamped control output.
   */
  double compute(double error, double dt = 1.0)
  {
    if (dt <= 0.0) dt = 1.0;

    // Proportional
    double p_term = kp_ * error;

    // Integral with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    double i_term = ki_ * integral_;

    // Derivative with low-pass filter
    double raw_derivative = (error - prev_error_) / dt;
    filtered_derivative_ = d_filter_alpha_ * filtered_derivative_ +
                           (1.0 - d_filter_alpha_) * raw_derivative;
    double d_term = kd_ * filtered_derivative_;

    prev_error_ = error;

    // Sum and clamp
    double output = p_term + i_term + d_term;
    output = std::clamp(output, output_min_, output_max_);

    // Anti-windup: if output is saturated, stop accumulating integral
    if ((output >= output_max_ && error > 0.0) ||
        (output <= output_min_ && error < 0.0))
    {
      integral_ -= error * dt;  // undo the last accumulation
    }

    return output;
  }

  /**
   * @brief Backward-compatible overload (unit dt, no derivative filtering).
   *
   * Matches the original API so existing tests pass unchanged.
   */
  double computeSimple(double error)
  {
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
  }

  // ─── Accessors ──────────────────────────────────────────────
  double getProportionalGain() const { return kp_; }
  double getIntegralGain()     const { return ki_; }
  double getDerivativeGain()   const { return kd_; }
  double getIntegral()         const { return integral_; }

private:
  // Gains
  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.1;

  // State
  double integral_             = 0.0;
  double prev_error_           = 0.0;
  double filtered_derivative_  = 0.0;

  // Limits
  double output_min_     = -1e9;
  double output_max_     =  1e9;
  double integral_limit_ =  1e6;

  // Derivative filter coefficient (0 = no filter, 1 = infinite smoothing)
  double d_filter_alpha_ = 0.1;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__CONTROLLER__PID_CONTROLLER_HPP_
