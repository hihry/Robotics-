/**
 * @file trajectory_generator.cpp
 * @brief Implementation of trajectory generation with velocity profiling.
 *
 * Converts a smooth geometric path into a time-parameterized trajectory
 * using either constant or trapezoidal velocity profiles.
 */

#include "trajectory_control/trajectory_generator.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>

namespace trajectory_control
{

TrajectoryGenerator::TrajectoryGenerator() = default;

void TrajectoryGenerator::setMaxVelocity(double v_max)
{
  if (v_max <= 0.0) {
    throw std::invalid_argument("Max velocity must be positive");
  }
  v_max_ = v_max;
}

void TrajectoryGenerator::setMaxAcceleration(double a_max)
{
  if (a_max <= 0.0) {
    throw std::invalid_argument("Max acceleration must be positive");
  }
  a_max_ = a_max;
}

void TrajectoryGenerator::setProfileType(VelocityProfileType type)
{
  profile_type_ = type;
}

void TrajectoryGenerator::setTimeStep(double dt)
{
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive");
  }
  dt_ = dt;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generate(
  const std::vector<Pose2D> & path) const
{
  // --- Input validation ---
  if (path.size() < 2) {
    throw std::invalid_argument(
      "TrajectoryGenerator::generate() requires at least 2 path points, got " +
      std::to_string(path.size()));
  }

  // Step 1: Compute cumulative arc lengths
  std::vector<double> arc_lengths = computeArcLengths(path);
  double total_length = arc_lengths.back();

  if (total_length < 1e-9) {
    throw std::invalid_argument("Path has zero length (all points are identical)");
  }

  // Step 2: Generate trajectory by advancing along the path
  std::vector<TrajectoryPoint> trajectory;

  double current_s = 0.0;   // current arc-length position
  double current_t = 0.0;   // current time
  double current_v = 0.0;   // current velocity

  while (current_s < total_length) {
    // Compute desired velocity based on profile type
    double desired_v = 0.0;
    if (profile_type_ == VelocityProfileType::TRAPEZOIDAL) {
      desired_v = trapezoidalVelocity(current_s, total_length);
    } else {
      desired_v = v_max_;  // Constant velocity
    }

    // Apply acceleration limits
    double dv = desired_v - current_v;
    double max_dv = a_max_ * dt_;
    if (std::abs(dv) > max_dv) {
      current_v += (dv > 0 ? max_dv : -max_dv);
    } else {
      current_v = desired_v;
    }
    current_v = std::max(0.01, std::min(current_v, v_max_));  // Clamp

    // Interpolate pose at current arc length
    Pose2D pose = interpolateAtArcLength(path, arc_lengths, current_s);

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

    // Create trajectory point
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

    // Advance position and time
    double ds = current_v * dt_;
    current_s += ds;
    current_t += dt_;
  }

  // Ensure the last trajectory point matches the path endpoint
  if (!trajectory.empty()) {
    TrajectoryPoint last_tp;
    last_tp.x = path.back().x;
    last_tp.y = path.back().y;
    last_tp.theta = path.back().theta;
    last_tp.v = 0.0;  // Stop at goal
    last_tp.omega = 0.0;
    last_tp.curvature = 0.0;
    last_tp.timestamp = current_t;
    last_tp.arc_length = total_length;
    trajectory.push_back(last_tp);
  }

  return trajectory;
}

double TrajectoryGenerator::trapezoidalVelocity(double s, double total_length) const
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
   * s1 = distance to accelerate from 0 to v_max = v_max^2 / (2*a_max)
   * s3 = distance to decelerate from v_max to 0 = v_max^2 / (2*a_max)
   * If path is too short for full trapezoid, use triangular profile.
   */

  double accel_dist = (v_max_ * v_max_) / (2.0 * a_max_);
  double decel_dist = accel_dist;

  if (accel_dist + decel_dist > total_length) {
    // Triangular profile: path too short for full cruise phase
    double half = total_length / 2.0;
    double v_peak = std::sqrt(a_max_ * total_length);
    v_peak = std::min(v_peak, v_max_);

    if (s < half) {
      // Acceleration phase
      return std::sqrt(std::max(0.0, 2.0 * a_max_ * s));
    } else {
      // Deceleration phase
      double remaining = total_length - s;
      return std::sqrt(std::max(0.0, 2.0 * a_max_ * remaining));
    }
  }

  double s_cruise_start = accel_dist;
  double s_cruise_end = total_length - decel_dist;

  if (s < s_cruise_start) {
    // Acceleration phase: v = sqrt(2 * a_max * s)
    return std::sqrt(std::max(0.0, 2.0 * a_max_ * s));
  } else if (s < s_cruise_end) {
    // Cruise phase
    return v_max_;
  } else {
    // Deceleration phase: v = sqrt(2 * a_max * (total - s))
    double remaining = total_length - s;
    return std::sqrt(std::max(0.0, 2.0 * a_max_ * remaining));
  }
}

double TrajectoryGenerator::computeCurvature(
  const std::vector<Pose2D> & path, size_t index) const
{
  /**
   * Curvature using finite differences:
   *   κ = (x' * y'' - y' * x'') / (x'^2 + y'^2)^(3/2)
   *
   * Using central differences where possible.
   */
  if (path.size() < 3 || index == 0 || index >= path.size() - 1) {
    return 0.0;
  }

  const auto & prev = path[index - 1];
  const auto & curr = path[index];
  const auto & next = path[index + 1];

  double dx = (next.x - prev.x) / 2.0;
  double dy = (next.y - prev.y) / 2.0;
  double ddx = next.x - 2.0 * curr.x + prev.x;
  double ddy = next.y - 2.0 * curr.y + prev.y;

  double denom = std::pow(dx * dx + dy * dy, 1.5);
  if (denom < 1e-12) {
    return 0.0;
  }

  return (dx * ddy - dy * ddx) / denom;
}

std::vector<double> TrajectoryGenerator::computeArcLengths(
  const std::vector<Pose2D> & path) const
{
  std::vector<double> arc(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] + path[i - 1].distanceTo(path[i]);
  }
  return arc;
}

double TrajectoryGenerator::computeTotalArcLength(
  const std::vector<Pose2D> & path) const
{
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total += path[i - 1].distanceTo(path[i]);
  }
  return total;
}

Pose2D TrajectoryGenerator::interpolateAtArcLength(
  const std::vector<Pose2D> & path,
  const std::vector<double> & arc_lengths,
  double target_s) const
{
  // Clamp to valid range
  target_s = std::max(0.0, std::min(target_s, arc_lengths.back()));

  // Find segment
  size_t seg = 0;
  for (size_t i = 0; i < arc_lengths.size() - 1; ++i) {
    if (target_s <= arc_lengths[i + 1]) {
      seg = i;
      break;
    }
    seg = i;
  }

  double seg_len = arc_lengths[seg + 1] - arc_lengths[seg];
  double t = (seg_len > 1e-12) ? (target_s - arc_lengths[seg]) / seg_len : 0.0;
  t = std::max(0.0, std::min(t, 1.0));

  Pose2D result;
  result.x = path[seg].x + t * (path[seg + 1].x - path[seg].x);
  result.y = path[seg].y + t * (path[seg + 1].y - path[seg].y);
  result.theta = std::atan2(
    path[seg + 1].y - path[seg].y,
    path[seg + 1].x - path[seg].x);

  return result;
}

}  // namespace trajectory_control
