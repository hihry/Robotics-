/**
 * @file trapezoidal_velocity_generator.cpp
 * @brief Trapezoidal velocity profile trajectory generation.
 *
 * Converts a smoothed geometric path into a time-parameterized trajectory
 * with three-phase velocity: acceleration → cruise → deceleration.
 */

#include "smooth_nav_core/trajectory_generator/trapezoidal_velocity_generator.hpp"
#include "smooth_nav_core/trajectory_generator/trajectory_utils.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace smooth_nav_core
{

void TrapezoidalVelocityGenerator::setMaxVelocity(double v_max)
{
  if (v_max <= 0.0) {
    throw std::invalid_argument("Max velocity must be positive");
  }
  v_max_ = v_max;
}

void TrapezoidalVelocityGenerator::setMaxAcceleration(double a_max)
{
  if (a_max <= 0.0) {
    throw std::invalid_argument("Max acceleration must be positive");
  }
  a_max_ = a_max;
}

void TrapezoidalVelocityGenerator::setTimeStep(double dt)
{
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive");
  }
  dt_ = dt;
}

std::vector<TrajectoryPoint> TrapezoidalVelocityGenerator::generate(
  const std::vector<PathPoint> & path) const
{
  if (path.size() < 2) {
    throw std::invalid_argument(
      "TrapezoidalVelocityGenerator::generate() requires >= 2 path points, got " +
      std::to_string(path.size()));
  }

  // Step 1: Compute cumulative arc lengths
  std::vector<double> arc_lengths = computePathArcLengths(path);
  double total_length = arc_lengths.back();

  if (total_length < 1e-9) {
    throw std::invalid_argument("Path has zero length (all points are identical)");
  }

  // Step 2: Generate trajectory by advancing along the path
  std::vector<TrajectoryPoint> trajectory;

  double current_s = 0.0;
  double current_t = 0.0;
  double current_v = 0.0;

  while (current_s < total_length) {
    // Compute desired velocity from trapezoidal profile
    double desired_v = trapezoidalVelocity(current_s, total_length);

    // Apply acceleration limits
    double dv = desired_v - current_v;
    double max_dv = a_max_ * dt_;
    if (std::abs(dv) > max_dv) {
      current_v += (dv > 0 ? max_dv : -max_dv);
    } else {
      current_v = desired_v;
    }
    current_v = std::max(0.01, std::min(current_v, v_max_));

    // Interpolate pose at current arc length
    PathPoint pose = interpolateAtArcLength(path, arc_lengths, current_s);

    // Compute curvature at nearest path point
    size_t nearest_idx = 0;
    for (size_t i = 0; i < arc_lengths.size() - 1; ++i) {
      if (current_s <= arc_lengths[i + 1]) {
        nearest_idx = i;
        break;
      }
      nearest_idx = i;
    }
    double curvature = computeCurvature(path, nearest_idx);

    // Compute angular velocity: ω = v * κ
    double omega = current_v * curvature;

    TrajectoryPoint tp;
    tp.x = pose.x;
    tp.y = pose.y;
    tp.theta = pose.theta;
    tp.v = current_v;
    tp.omega = omega;
    tp.curvature = curvature;
    tp.timestamp = current_t;
    tp.arc_length = current_s;

    trajectory.push_back(tp);

    current_s += current_v * dt_;
    current_t += dt_;
  }

  // Final point at path endpoint with zero velocity
  if (!trajectory.empty()) {
    TrajectoryPoint last_tp;
    last_tp.x = path.back().x;
    last_tp.y = path.back().y;
    last_tp.theta = path.back().theta;
    last_tp.v = 0.0;
    last_tp.omega = 0.0;
    last_tp.curvature = 0.0;
    last_tp.timestamp = current_t;
    last_tp.arc_length = total_length;
    trajectory.push_back(last_tp);
  }

  return trajectory;
}

double TrapezoidalVelocityGenerator::trapezoidalVelocity(
  double s, double total_length) const
{
  /**
   * Trapezoidal Velocity Profile:
   *
   *   v ^
   *     |   /‾‾‾‾‾‾‾‾\
   *     |  /            \
   *     | /              \
   *     |/________________\___> s
   *     0   s1    s2    total_length
   *
   * s1 = v_max^2 / (2*a_max)
   * If path too short for full trapezoid → triangular profile.
   */
  double accel_dist = (v_max_ * v_max_) / (2.0 * a_max_);
  double decel_dist = accel_dist;

  if (accel_dist + decel_dist > total_length) {
    // Triangular profile
    double half = total_length / 2.0;
    if (s < half) {
      return std::sqrt(std::max(0.0, 2.0 * a_max_ * s));
    } else {
      double remaining = total_length - s;
      return std::sqrt(std::max(0.0, 2.0 * a_max_ * remaining));
    }
  }

  double s_cruise_start = accel_dist;
  double s_cruise_end = total_length - decel_dist;

  if (s < s_cruise_start) {
    return std::sqrt(std::max(0.0, 2.0 * a_max_ * s));
  } else if (s < s_cruise_end) {
    return v_max_;
  } else {
    double remaining = total_length - s;
    return std::sqrt(std::max(0.0, 2.0 * a_max_ * remaining));
  }
}

}  // namespace smooth_nav_core
