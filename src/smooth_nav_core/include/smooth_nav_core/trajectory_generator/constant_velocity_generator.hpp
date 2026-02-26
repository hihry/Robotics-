/**
 * @file constant_velocity_generator.hpp
 * @brief Trajectory generator with constant velocity profile.
 *
 * The robot moves at v_max along the entire path (no accel/decel phases).
 * Optionally limits speed in curves via centripetal acceleration bound.
 * Useful as a baseline for comparison with the trapezoidal profile.
 */

#ifndef SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__CONSTANT_VELOCITY_GENERATOR_HPP_
#define SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__CONSTANT_VELOCITY_GENERATOR_HPP_

#include "smooth_nav_core/trajectory_generator/i_trajectory_generator.hpp"
#include <vector>

namespace smooth_nav_core
{

class ConstantVelocityGenerator : public ITrajectoryGenerator
{
public:
  ConstantVelocityGenerator() = default;
  ~ConstantVelocityGenerator() override = default;

  void setMaxVelocity(double v_max);
  void setTimeStep(double dt);
  void setMaxLateralAcceleration(double a_lat);

  std::vector<TrajectoryPoint> generate(
    const std::vector<PathPoint> & path) const override;

  std::string name() const override { return "constant"; }

private:
  double v_max_     = 0.18;
  double dt_        = 0.05;
  double a_lat_max_ = 10.0;  ///< effectively unlimited by default
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__CONSTANT_VELOCITY_GENERATOR_HPP_
