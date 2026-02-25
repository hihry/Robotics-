/**
 * @file pid_controller.hpp
 * @brief Reusable PID controller utility class.
 *
 * Standalone PID — used by PurePursuitController for cross-track correction
 * and heading correction.
 */

#ifndef SMOOTH_NAV_CORE__CONTROLLER__PID_CONTROLLER_HPP_
#define SMOOTH_NAV_CORE__CONTROLLER__PID_CONTROLLER_HPP_

namespace smooth_nav_core
{

class PIDController
{
public:
  PIDController() = default;
  PIDController(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd) {}

  void setGains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  /**
   * @brief Compute PID output for a given error.
   *
   * @param error Current error value.
   * @return Control output.
   */
  double compute(double error)
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
  }

private:
  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.1;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__CONTROLLER__PID_CONTROLLER_HPP_
