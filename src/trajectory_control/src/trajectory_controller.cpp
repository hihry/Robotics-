/**
 * @file trajectory_controller.cpp
 * @brief Pure Pursuit + PID trajectory tracking controller implementation.
 *
 * Algorithm flow:
 *   1. Find nearest trajectory point to robot (cross-track reference)
 *   2. Find look-ahead point at distance L_d ahead
 *   3. Compute steering curvature: κ = 2*sin(α)/L_d
 *   4. Compute angular velocity: ω = v * κ
 *   5. Apply PID correction on velocity error
 *   6. Clamp outputs to robot limits
 *
 * Reference:
 *   R. Craig Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm"
 *   CMU-RI-TR-92-01, Jan 1992
 */

#include "trajectory_control/trajectory_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace trajectory_control
{

TrajectoryController::TrajectoryController() = default;

void TrajectoryController::setLookAheadDistance(double distance)
{
  if (distance <= 0.0) {
    throw std::invalid_argument("Look-ahead distance must be positive");
  }
  look_ahead_distance_ = distance;
}

void TrajectoryController::setMaxLinearVelocity(double v_max)
{
  max_linear_vel_ = v_max;
}

void TrajectoryController::setMaxAngularVelocity(double omega_max)
{
  max_angular_vel_ = omega_max;
}

void TrajectoryController::setGoalTolerance(double tolerance)
{
  goal_tolerance_ = tolerance;
}

void TrajectoryController::setPIDGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void TrajectoryController::setTrajectory(const std::vector<TrajectoryPoint> & trajectory)
{
  trajectory_ = trajectory;
  trajectory_set_ = !trajectory.empty();
  trajectory_complete_ = false;
  nearest_index_ = 0;
  integral_error_ = 0.0;
  prev_error_ = 0.0;
}

void TrajectoryController::reset()
{
  trajectory_.clear();
  trajectory_set_ = false;
  trajectory_complete_ = false;
  nearest_index_ = 0;
  cross_track_error_ = 0.0;
  heading_error_ = 0.0;
  integral_error_ = 0.0;
  prev_error_ = 0.0;
}

bool TrajectoryController::isTrajectoryComplete() const
{
  return trajectory_complete_;
}

double TrajectoryController::getCrossTrackError() const
{
  return cross_track_error_;
}

double TrajectoryController::getHeadingError() const
{
  return heading_error_;
}

size_t TrajectoryController::getNearestIndex() const
{
  return nearest_index_;
}

std::pair<double, double> TrajectoryController::computeControl(
  const Pose2D & current_pose,
  double /*current_time*/)
{
  // --- Guard clauses ---
  if (!trajectory_set_ || trajectory_.empty()) {
    return {0.0, 0.0};
  }

  if (trajectory_complete_) {
    return {0.0, 0.0};
  }

  // === Step 1: Find nearest trajectory point ===
  nearest_index_ = findNearestPoint(current_pose);

  // Compute cross-track error
  const auto & nearest = trajectory_[nearest_index_];
  cross_track_error_ = std::hypot(current_pose.x - nearest.x,
                                   current_pose.y - nearest.y);

  // === Step 2: Check if trajectory is complete ===
  const auto & goal = trajectory_.back();
  double dist_to_goal = std::hypot(current_pose.x - goal.x,
                                    current_pose.y - goal.y);
  if (dist_to_goal < goal_tolerance_) {
    trajectory_complete_ = true;
    return {0.0, 0.0};
  }

  // === Step 3: Find look-ahead point ===
  size_t look_ahead_idx = findLookAheadPoint(current_pose, nearest_index_);
  const auto & look_ahead = trajectory_[look_ahead_idx];

  // === Step 4: Pure Pursuit — Compute steering ===
  // Transform look-ahead point to robot's local frame
  double dx = look_ahead.x - current_pose.x;
  double dy = look_ahead.y - current_pose.y;

  // Rotate into robot frame
  double local_x = dx * std::cos(current_pose.theta) + dy * std::sin(current_pose.theta);
  double local_y = -dx * std::sin(current_pose.theta) + dy * std::cos(current_pose.theta);

  // Actual look-ahead distance (Euclidean to look-ahead point)
  double actual_Ld = std::hypot(local_x, local_y);
  if (actual_Ld < 1e-6) {
    actual_Ld = look_ahead_distance_;
  }

  // Pure Pursuit curvature: κ = 2*y_local / L_d^2
  double curvature = 2.0 * local_y / (actual_Ld * actual_Ld);

  // === Step 5: Compute velocity commands ===
  // Use trajectory's desired velocity with PID correction
  double desired_v = trajectory_[nearest_index_].v;
  if (desired_v < 0.01) desired_v = 0.05;  // Minimum velocity to keep moving

  // PID on cross-track error (reduce speed when error is large)
  double error = cross_track_error_;
  integral_error_ += error;
  double derivative = error - prev_error_;
  prev_error_ = error;

  double pid_correction = kp_ * error + ki_ * integral_error_ + kd_ * derivative;

  // Reduce speed proportionally to cross-track error
  double v_cmd = desired_v - std::abs(pid_correction) * 0.5;
  v_cmd = std::max(0.02, std::min(v_cmd, max_linear_vel_));

  // Angular velocity: ω = v * κ
  double omega_cmd = v_cmd * curvature;

  // === Step 6: Clamp to robot limits ===
  v_cmd = std::max(-max_linear_vel_, std::min(v_cmd, max_linear_vel_));
  omega_cmd = std::max(-max_angular_vel_, std::min(omega_cmd, max_angular_vel_));

  // Compute heading error for logging
  double desired_theta = std::atan2(dy, dx);
  heading_error_ = normalizeAngle(desired_theta - current_pose.theta);

  return {v_cmd, omega_cmd};
}

size_t TrajectoryController::findNearestPoint(const Pose2D & pose) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t nearest = nearest_index_;  // Start from last known nearest

  // Search in a window around the last nearest index (efficiency optimization)
  size_t search_start = (nearest_index_ > 10) ? nearest_index_ - 10 : 0;
  size_t search_end = std::min(nearest_index_ + 50, trajectory_.size());

  for (size_t i = search_start; i < search_end; ++i) {
    double dist = std::hypot(pose.x - trajectory_[i].x,
                              pose.y - trajectory_[i].y);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = i;
    }
  }

  return nearest;
}

size_t TrajectoryController::findLookAheadPoint(
  const Pose2D & pose, size_t nearest_idx) const
{
  // Search forward from nearest point to find look-ahead point
  for (size_t i = nearest_idx; i < trajectory_.size(); ++i) {
    double dist = std::hypot(pose.x - trajectory_[i].x,
                              pose.y - trajectory_[i].y);
    if (dist >= look_ahead_distance_) {
      return i;
    }
  }

  // If no point far enough, use the last point
  return trajectory_.size() - 1;
}

double TrajectoryController::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace trajectory_control
