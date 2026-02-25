/**
 * @file trapezoidal_velocity_generator.hpp
 * @brief Trajectory generator with trapezoidal velocity profile.
 *
 * Three phases: acceleration -> cruise -> deceleration.
 * Handles triangular profile when path is too short for full cruise phase.
 */

#ifndef SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAPEZOIDAL_VELOCITY_GENERATOR_HPP_
#define SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAPEZOIDAL_VELOCITY_GENERATOR_HPP_

#include "smooth_nav_core/trajectory_generator/i_trajectory_generator.hpp"
#include "smooth_nav_core/trajectory_generator/trajectory_utils.hpp"
#include <vector>

namespace smooth_nav_core
{

class TrapezoidalVelocityGenerator : public ITrajectoryGenerator
{
public:
  TrapezoidalVelocityGenerator() = default;
  ~TrapezoidalVelocityGenerator() override = default;

  void setMaxVelocity(double v_max);
  void setMaxAcceleration(double a_max);
  void setTimeStep(double dt);

  std::vector<TrajectoryPoint> generate(
    const std::vector<PathPoint> & path) const override;

  std::string name() const override { return "trapezoidal"; }

private:
  double v_max_ = 0.18;
  double a_max_ = 0.5;
  double dt_ = 0.05;

  double trapezoidalVelocity(double s, double total_length) const;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAPEZOIDAL_VELOCITY_GENERATOR_HPP_
