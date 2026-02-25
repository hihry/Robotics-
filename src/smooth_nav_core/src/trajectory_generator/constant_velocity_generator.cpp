/**
 * @file constant_velocity_generator.cpp
 * @brief Constant velocity trajectory generation.
 *
 * Simple profile: robot moves at v_max along the entire path.
 * Useful as a baseline for comparison with trapezoidal profile.
 */

#include "smooth_nav_core/trajectory_generator/constant_velocity_generator.hpp"
#include "smooth_nav_core/trajectory_generator/trajectory_utils.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace smooth_nav_core
{

void ConstantVelocityGenerator::setMaxVelocity(double v_max)
{
  if (v_max <= 0.0) {
    throw std::invalid_argument("Max velocity must be positive");
  }
  v_max_ = v_max;
}

void ConstantVelocityGenerator::setTimeStep(double dt)
{
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive");
  }
  dt_ = dt;
}

std::vector<TrajectoryPoint> ConstantVelocityGenerator::generate(
  const std::vector<PathPoint> & path) const
{
  if (path.size() < 2) {
    throw std::invalid_argument(
      "ConstantVelocityGenerator::generate() requires >= 2 path points, got " +
      std::to_string(path.size()));
  }

  std::vector<double> arc_lengths = computePathArcLengths(path);
  double total_length = arc_lengths.back();

  if (total_length < 1e-9) {
    throw std::invalid_argument("Path has zero length (all points are identical)");
  }

  std::vector<TrajectoryPoint> trajectory;

  double current_s = 0.0;
  double current_t = 0.0;

  while (current_s < total_length) {
    PathPoint pose = interpolateAtArcLength(path, arc_lengths, current_s);

    size_t nearest_idx = 0;
    for (size_t i = 0; i < arc_lengths.size() - 1; ++i) {
      if (current_s <= arc_lengths[i + 1]) {
        nearest_idx = i;
        break;
      }
      nearest_idx = i;
    }
    double curvature = computeCurvature(path, nearest_idx);
    double omega = v_max_ * curvature;

    TrajectoryPoint tp;
    tp.x = pose.x;
    tp.y = pose.y;
    tp.theta = pose.theta;
    tp.v = v_max_;
    tp.omega = omega;
    tp.curvature = curvature;
    tp.timestamp = current_t;
    tp.arc_length = current_s;

    trajectory.push_back(tp);

    current_s += v_max_ * dt_;
    current_t += dt_;
  }

  // Final point
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

}  // namespace smooth_nav_core
