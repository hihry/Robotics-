/**
 * @file pure_pursuit_controller.cpp
 * @brief Pure Pursuit + PID trajectory tracking controller.
 *
 * Algorithm:
 *   1. Find nearest trajectory point to robot (cross-track reference)
 *   2. Find look-ahead point at distance L_d ahead
 *   3. Compute steering curvature: κ = 2*sin(α)/L_d
 *   4. Compute angular velocity: ω = v * κ
 *   5. Apply PID correction on cross-track error
 *   6. Clamp outputs to robot limits
 *
 * Reference:
 *   R. Craig Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm"
 *   CMU-RI-TR-92-01, Jan 1992
 */

#include "smooth_nav_core/controller/pure_pursuit_controller.hpp"
#include "smooth_nav_core/math/geometry_utils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace smooth_nav_core
{

// ============ Configuration ============

void PurePursuitController::setLookAheadDistance(double distance)
{
  if (distance <= 0.0) {
    throw std::invalid_argument("Look-ahead distance must be positive");
  }
  look_ahead_distance_ = distance;
}

void PurePursuitController::setMaxLinearVelocity(double v_max)
{
  max_linear_vel_ = v_max;
}

void PurePursuitController::setMaxAngularVelocity(double omega_max)
{
  max_angular_vel_ = omega_max;
}

void PurePursuitController::setGoalTolerance(double tolerance)
{
  goal_tolerance_ = tolerance;
}

void PurePursuitController::setPIDGains(double kp, double ki, double kd)
{
  pid_.setGains(kp, ki, kd);
}

// ============ IController Interface ============

void PurePursuitController::setTrajectory(
  const std::vector<TrajectoryPoint> & trajectory)
{
  trajectory_ = trajectory;
  trajectory_set_ = !trajectory.empty();
  trajectory_complete_ = false;
  nearest_index_ = 0;
  cross_track_error_ = 0.0;
  heading_error_ = 0.0;
  pid_.reset();
}

Velocity2D PurePursuitController::computeControl(
  const Pose2D & current_pose,
  double /*time*/)
{
  // Guard clauses
  if (!trajectory_set_ || trajectory_.empty()) {
    return {0.0, 0.0};
  }
  if (trajectory_complete_) {
    return {0.0, 0.0};
  }

  // === Step 1: Find nearest trajectory point ===
  nearest_index_ = findNearestPoint(current_pose);

  const auto & nearest = trajectory_[nearest_index_];
  cross_track_error_ = std::hypot(current_pose.x - nearest.x,
                                   current_pose.y - nearest.y);

  // === Step 2: Check goal reached ===
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

  // === Step 4: Pure Pursuit steering ===
  double dx = look_ahead.x - current_pose.x;
  double dy = look_ahead.y - current_pose.y;

  // Transform to robot-local frame
  double local_x =  dx * std::cos(current_pose.theta) + dy * std::sin(current_pose.theta);
  double local_y = -dx * std::sin(current_pose.theta) + dy * std::cos(current_pose.theta);

  double actual_Ld = std::hypot(local_x, local_y);
  if (actual_Ld < 1e-6) {
    actual_Ld = look_ahead_distance_;
  }

  // Pure Pursuit curvature: κ = 2 * y_local / L_d²
  double curvature = 2.0 * local_y / (actual_Ld * actual_Ld);

  // === Step 5: Compute velocity commands ===
  double desired_v = trajectory_[nearest_index_].v;
  if (desired_v < 0.01) desired_v = 0.05;

  // PID on cross-track error → reduce speed when error is large
  double pid_output = pid_.compute(cross_track_error_);
  double v_cmd = desired_v - std::abs(pid_output) * 0.5;
  v_cmd = std::max(0.02, std::min(v_cmd, max_linear_vel_));

  // Angular velocity: ω = v * κ
  double omega_cmd = v_cmd * curvature;

  // === Step 6: Clamp to robot limits ===
  v_cmd = std::max(-max_linear_vel_, std::min(v_cmd, max_linear_vel_));
  omega_cmd = std::max(-max_angular_vel_, std::min(omega_cmd, max_angular_vel_));

  // Heading error for diagnostics
  double desired_theta = std::atan2(dy, dx);
  heading_error_ = normalizeAngle(desired_theta - current_pose.theta);

  return {v_cmd, omega_cmd};
}

bool PurePursuitController::isComplete() const
{
  return trajectory_complete_;
}

ControllerState PurePursuitController::getState() const
{
  ControllerState state;
  if (!trajectory_.empty() && nearest_index_ < trajectory_.size()) {
    state.robot_pose = Pose2D(trajectory_[nearest_index_].x,
                               trajectory_[nearest_index_].y,
                               trajectory_[nearest_index_].theta);
  }
  state.cross_track_error = cross_track_error_;
  state.heading_error = heading_error_;
  state.nearest_index = nearest_index_;
  state.trajectory_complete = trajectory_complete_;
  return state;
}

void PurePursuitController::reset()
{
  trajectory_.clear();
  trajectory_set_ = false;
  trajectory_complete_ = false;
  nearest_index_ = 0;
  cross_track_error_ = 0.0;
  heading_error_ = 0.0;
  pid_.reset();
}

// ============ Diagnostics ============

double PurePursuitController::getCrossTrackError() const
{
  return cross_track_error_;
}

double PurePursuitController::getHeadingError() const
{
  return heading_error_;
}

size_t PurePursuitController::getNearestIndex() const
{
  return nearest_index_;
}

// ============ Private Helpers ============

size_t PurePursuitController::findNearestPoint(const Pose2D & pose) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t nearest = nearest_index_;

  // Windowed search around last known index for efficiency
  size_t search_start = (nearest_index_ > 10) ? nearest_index_ - 10 : 0;
  size_t search_end = std::min(nearest_index_ + 50, trajectory_.size());

  for (size_t i = search_start; i < search_end; ++i) {
    double d = std::hypot(pose.x - trajectory_[i].x,
                           pose.y - trajectory_[i].y);
    if (d < min_dist) {
      min_dist = d;
      nearest = i;
    }
  }

  return nearest;
}

size_t PurePursuitController::findLookAheadPoint(
  const Pose2D & pose, size_t nearest_idx) const
{
  for (size_t i = nearest_idx; i < trajectory_.size(); ++i) {
    double d = std::hypot(pose.x - trajectory_[i].x,
                           pose.y - trajectory_[i].y);
    if (d >= look_ahead_distance_) {
      return i;
    }
  }
  return trajectory_.size() - 1;
}

}  // namespace smooth_nav_core
