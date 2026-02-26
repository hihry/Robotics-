/**
 * @file trapezoidal_velocity_generator.cpp
 * @brief Trapezoidal velocity profile with curvature-based speed limiting.
 *
 * Converts a smoothed geometric path into a time-parameterized trajectory.
 *
 * Velocity envelope:
 *   1. Trapezoidal (accel → cruise → decel) based on arc-length position.
 *   2. Curvature limit: v ≤ √(a_lat_max / |κ|) — slows in tight curves.
 *   3. Acceleration rate-limit: |Δv| ≤ a_max · dt per step.
 *   4. Minimum velocity floor (0.01 m/s) — prevents stalling mid-path.
 *
 * The tighter of the trapezoidal and curvature limits wins at each step.
 */

#include "smooth_nav_core/trajectory_generator/trapezoidal_velocity_generator.hpp"
#include "smooth_nav_core/trajectory_generator/trajectory_utils.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace smooth_nav_core
{

// ─── Setters ─────────────────────────────────────────────────────────────────

void TrapezoidalVelocityGenerator::setMaxVelocity(double v_max)
{
  if (v_max <= 0.0) throw std::invalid_argument("Max velocity must be positive");
  v_max_ = v_max;
}

void TrapezoidalVelocityGenerator::setMaxAcceleration(double a_max)
{
  if (a_max <= 0.0) throw std::invalid_argument("Max acceleration must be positive");
  a_max_ = a_max;
}

void TrapezoidalVelocityGenerator::setMaxLateralAcceleration(double a_lat_max)
{
  if (a_lat_max <= 0.0) throw std::invalid_argument("Max lateral acceleration must be positive");
  a_lat_max_ = a_lat_max;
}

void TrapezoidalVelocityGenerator::setTimeStep(double dt)
{
  if (dt <= 0.0) throw std::invalid_argument("Time step must be positive");
  dt_ = dt;
}

// ─── Main generation ─────────────────────────────────────────────────────────

std::vector<TrajectoryPoint> TrapezoidalVelocityGenerator::generate(
  const std::vector<PathPoint> & path) const
{
  if (path.size() < 2) {
    throw std::invalid_argument(
      "TrapezoidalVelocityGenerator::generate() requires >= 2 path points, got " +
      std::to_string(path.size()));
  }

  // Step 1: cumulative arc lengths
  std::vector<double> arc_lengths = computePathArcLengths(path);
  double total_length = arc_lengths.back();

  if (total_length < 1e-9) {
    throw std::invalid_argument("Path has zero length (all points are identical)");
  }

  // ─── Forward integration along the path ────────────────────────────────
  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(static_cast<size_t>(total_length / (v_max_ * dt_)) + 10);

  double current_s = 0.0;
  double current_t = 0.0;
  double current_v = 0.0;

  while (current_s < total_length) {
    // Interpolate pose at current arc length
    PathPoint pose = interpolateAtArcLength(path, arc_lengths, current_s);

    // Curvature at this position (from the smoothed path)
    size_t nearest_idx = 0;
    for (size_t i = 0; i < arc_lengths.size() - 1; ++i) {
      if (current_s <= arc_lengths[i + 1]) { nearest_idx = i; break; }
      nearest_idx = i;
    }
    double curvature = computeCurvature(path, nearest_idx);

    // Desired velocity = min(trapezoidal envelope, curvature limit)
    double v_trap = trapezoidalVelocity(current_s, total_length);
    double v_curv = curvatureLimitedVelocity(curvature);
    double desired_v = std::min(v_trap, v_curv);

    // Rate-limit acceleration
    double dv     = desired_v - current_v;
    double max_dv = a_max_ * dt_;
    if (std::abs(dv) > max_dv) {
      current_v += (dv > 0.0 ? max_dv : -max_dv);
    } else {
      current_v = desired_v;
    }
    current_v = std::max(0.01, std::min(current_v, v_max_));

    // Angular velocity: ω = v · κ
    double omega = current_v * curvature;

    TrajectoryPoint tp;
    tp.x          = pose.x;
    tp.y          = pose.y;
    tp.theta      = pose.theta;
    tp.v          = current_v;
    tp.omega      = omega;
    tp.curvature  = curvature;
    tp.timestamp  = current_t;
    tp.arc_length = current_s;
    trajectory.push_back(tp);

    current_s += current_v * dt_;
    current_t += dt_;
  }

  // ─── Final point at path endpoint with zero velocity ───────────────────
  if (!trajectory.empty()) {
    TrajectoryPoint last_tp;
    last_tp.x          = path.back().x;
    last_tp.y          = path.back().y;
    last_tp.theta      = path.back().theta;
    last_tp.v          = 0.0;
    last_tp.omega      = 0.0;
    last_tp.curvature  = 0.0;
    last_tp.timestamp  = current_t;
    last_tp.arc_length = total_length;
    trajectory.push_back(last_tp);
  }

  return trajectory;
}

// ─── Trapezoidal profile ─────────────────────────────────────────────────────

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
   *     +/________________\___> s
   *     0   s1    s2    L
   *
   *  s1 = v²/(2a)   s2 = L − s1
   *
   *  If s1 + s2 > L → triangular profile (never reaches v_max).
   */
  double accel_dist = (v_max_ * v_max_) / (2.0 * a_max_);
  double decel_dist = accel_dist;

  if (accel_dist + decel_dist > total_length) {
    // Triangular profile: peak at midpoint
    double half = total_length / 2.0;
    if (s < half) {
      return std::sqrt(std::max(0.0, 2.0 * a_max_ * s));
    } else {
      return std::sqrt(std::max(0.0, 2.0 * a_max_ * (total_length - s)));
    }
  }

  double s_cruise_start = accel_dist;
  double s_cruise_end   = total_length - decel_dist;

  if (s < s_cruise_start) {
    return std::sqrt(std::max(0.0, 2.0 * a_max_ * s));
  } else if (s < s_cruise_end) {
    return v_max_;
  } else {
    return std::sqrt(std::max(0.0, 2.0 * a_max_ * (total_length - s)));
  }
}

// ─── Curvature-based speed limit ─────────────────────────────────────────────

double TrapezoidalVelocityGenerator::curvatureLimitedVelocity(
  double curvature) const
{
  double abs_k = std::abs(curvature);
  if (abs_k < 1e-6) return v_max_;             // essentially straight
  double v_curv = std::sqrt(a_lat_max_ / abs_k);
  return std::min(v_curv, v_max_);
}

}  // namespace smooth_nav_core
