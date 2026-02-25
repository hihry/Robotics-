/**
 * @file trajectory_generator.hpp
 * @brief Generates time-parameterized trajectories from smooth paths.
 *
 * Takes a smooth path (from PathSmoother) and produces a trajectory
 * with timestamps, velocity profiles, and curvature information.
 *
 * Supports:
 *   - Constant velocity profile
 *   - Trapezoidal velocity profile (accel → cruise → decel)
 *
 * Reference:
 *   - Motion profiles: https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article
 */

#ifndef TRAJECTORY_CONTROL__TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_CONTROL__TRAJECTORY_GENERATOR_HPP_

#include "trajectory_control/types.hpp"
#include <vector>

namespace trajectory_control
{

/**
 * @class TrajectoryGenerator
 * @brief Converts a smooth geometric path into a time-parameterized trajectory.
 *
 * Usage:
 * @code
 *   TrajectoryGenerator gen;
 *   gen.setMaxVelocity(0.2);
 *   gen.setMaxAcceleration(0.5);
 *   gen.setProfileType(VelocityProfileType::TRAPEZOIDAL);
 *   auto trajectory = gen.generate(smooth_path);
 * @endcode
 */
class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  ~TrajectoryGenerator() = default;

  // ---- Configuration ----

  /// Set maximum linear velocity (m/s). TurtleBot3 Burger max: 0.22 m/s
  void setMaxVelocity(double v_max);

  /// Set maximum linear acceleration (m/s^2)
  void setMaxAcceleration(double a_max);

  /// Set the velocity profile type
  void setProfileType(VelocityProfileType type);

  /// Set time step for trajectory sampling (seconds)
  void setTimeStep(double dt);

  // ---- Core API ----

  /**
   * @brief Generate a time-parameterized trajectory from a smooth path.
   *
   * @param path Smooth path as a vector of Pose2D (from PathSmoother).
   *             Must have >= 2 points.
   * @return std::vector<TrajectoryPoint> Time-stamped trajectory.
   * @throws std::invalid_argument if path has fewer than 2 points.
   */
  std::vector<TrajectoryPoint> generate(const std::vector<Pose2D> & path) const;

  /**
   * @brief Get the total arc length of a path.
   */
  double computeTotalArcLength(const std::vector<Pose2D> & path) const;

private:
  double v_max_ = 0.18;     ///< Max linear velocity (m/s)
  double a_max_ = 0.5;      ///< Max linear acceleration (m/s^2)
  double dt_ = 0.05;        ///< Sampling time step (s)
  VelocityProfileType profile_type_ = VelocityProfileType::TRAPEZOIDAL;

  /**
   * @brief Compute velocity at a given arc-length position using trapezoidal profile.
   *
   * Three phases:
   *   1. Acceleration: v = a_max * t
   *   2. Cruise: v = v_max
   *   3. Deceleration: v = v_max - a_max * (t - t2)
   *
   * @param s Current arc length position
   * @param total_length Total path length
   * @return Velocity at position s
   */
  double trapezoidalVelocity(double s, double total_length) const;

  /**
   * @brief Compute curvature at a point on the path using finite differences.
   *
   * κ = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
   *
   * @param path The smooth path
   * @param index Point index
   * @return Curvature (1/m), positive = turning left
   */
  double computeCurvature(const std::vector<Pose2D> & path, size_t index) const;

  /**
   * @brief Compute cumulative arc lengths for the path.
   */
  std::vector<double> computeArcLengths(const std::vector<Pose2D> & path) const;

  /**
   * @brief Interpolate position on path at a given arc length.
   */
  Pose2D interpolateAtArcLength(
    const std::vector<Pose2D> & path,
    const std::vector<double> & arc_lengths,
    double target_s) const;
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TRAJECTORY_GENERATOR_HPP_
