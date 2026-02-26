/**
 * @file trapezoidal_velocity_generator.hpp
 * @brief Trajectory generator with trapezoidal velocity profile + curvature limiting.
 *
 * Three velocity phases: acceleration → cruise → deceleration.
 * Handles triangular profile automatically when the path is too short
 * to reach full cruise velocity.
 *
 * **Enhancement over a textbook trapezoidal profile:**
 *   The cruise velocity is reduced in high-curvature regions to satisfy
 *   the centripetal acceleration limit:  v ≤ √(a_lat_max / |κ|).
 *   This prevents excessive lateral acceleration in tight curves.
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
  void setMaxLateralAcceleration(double a_lat_max);
  void setTimeStep(double dt);

  std::vector<TrajectoryPoint> generate(
    const std::vector<PathPoint> & path) const override;

  std::string name() const override { return "trapezoidal"; }

private:
  double v_max_      = 0.18;   ///< m/s  — max cruise velocity
  double a_max_      = 0.5;    ///< m/s² — max longitudinal acceleration
  double a_lat_max_  = 0.4;    ///< m/s² — max centripetal (lateral) acceleration
  double dt_         = 0.05;   ///< s    — trajectory time step

  /**
   * @brief Compute the trapezoidal profile velocity at arc-length position s.
   *
   * Returns the *unconstrained* velocity from the accel/cruise/decel envelope.
   */
  double trapezoidalVelocity(double s, double total_length) const;

  /**
   * @brief Compute curvature-limited velocity at a given curvature.
   *
   *   v_curv = √(a_lat_max / |κ|)
   *
   * Returns v_max_ when curvature is negligible.
   */
  double curvatureLimitedVelocity(double curvature) const;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__TRAJECTORY_GENERATOR__TRAPEZOIDAL_VELOCITY_GENERATOR_HPP_
