/**
 * @file pure_pursuit_controller.cpp
 * @brief Pure Pursuit + PID trajectory tracking controller.
 *
 * Algorithm:
 *   1. Find nearest trajectory point (windowed search for O(1) amortised).
 *   2. Compute *signed* cross-track error (cross-product gives sign).
 *   3. Compute adaptive look-ahead distance: L_d = L_d_base + k · |v|.
 *   4. Find look-ahead point at distance L_d ahead on the trajectory.
 *   5. Pure Pursuit curvature: κ = 2 · y_local / L_d².
 *   6. Angular velocity: ω = v · κ   (+ PID correction on cross-track err).
 *   7. Goal-zone deceleration: linearly reduce speed inside decel_radius.
 *   8. Clamp to robot limits.
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

void PurePursuitController::setAdaptiveLookAheadGain(double k)
{
  adaptive_k_ = k;
}

void PurePursuitController::setGoalDecelerationRadius(double radius)
{
  goal_decel_radius_ = std::max(0.01, radius);
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
  prev_time_ = -1.0;
  pid_.reset();
}

Velocity2D PurePursuitController::computeControl(
  const Pose2D & current_pose,
  double time)
{
  // Guard clauses
  if (!trajectory_set_ || trajectory_.empty()) {
    return {0.0, 0.0};
  }
  if (trajectory_complete_) {
    return {0.0, 0.0};
  }

  // dt for PID
  double dt = 1.0;
  if (prev_time_ >= 0.0 && time > prev_time_) {
    dt = time - prev_time_;
  }
  prev_time_ = time;

  // === Step 1: Find nearest trajectory point ===
  nearest_index_ = findNearestPoint(current_pose);

  // === Step 2: Signed cross-track error ===
  cross_track_error_ = computeSignedCrossTrackError(current_pose, nearest_index_);

  // === Step 3: Check goal reached ===
  const auto & goal = trajectory_.back();
  double dist_to_goal = std::hypot(current_pose.x - goal.x,
                                    current_pose.y - goal.y);
  if (dist_to_goal < goal_tolerance_) {
    trajectory_complete_ = true;
    return {0.0, 0.0};
  }

  // === Step 4: Adaptive look-ahead distance ===
  double ref_v = std::abs(trajectory_[nearest_index_].v);
  double Ld = look_ahead_distance_ + adaptive_k_ * ref_v;
  Ld = std::max(Ld, 0.05);  // safety floor

  // === Step 5: Find look-ahead point ===
  size_t look_ahead_idx = findLookAheadPoint(current_pose, nearest_index_, Ld);
  const auto & look_ahead = trajectory_[look_ahead_idx];

  // === Step 6: Pure Pursuit steering ===
  double dx = look_ahead.x - current_pose.x;
  double dy = look_ahead.y - current_pose.y;

  // Transform to robot-local frame
  double cos_th = std::cos(current_pose.theta);
  double sin_th = std::sin(current_pose.theta);
  double local_x =  dx * cos_th + dy * sin_th;
  double local_y = -dx * sin_th + dy * cos_th;

  double actual_Ld = std::hypot(local_x, local_y);
  if (actual_Ld < 1e-6) {
    actual_Ld = Ld;
  }

  // Pure Pursuit curvature: κ = 2 · y_local / L_d²
  double curvature = 2.0 * local_y / (actual_Ld * actual_Ld);

  // === Step 7: Compute velocity commands ===
  // Feedforward: use trajectory reference velocity
  double desired_v = trajectory_[nearest_index_].v;
  if (desired_v < 0.01) desired_v = 0.05;

  // Goal deceleration
  if (dist_to_goal < goal_decel_radius_) {
    double scale = dist_to_goal / goal_decel_radius_;
    double min_approach_speed = 0.02;
    desired_v = min_approach_speed + scale * (desired_v - min_approach_speed);
  }

  // PID on signed cross-track error → angular correction
  double pid_correction = pid_.compute(cross_track_error_, dt);

  // Linear velocity: reduce when cross-track error is large
  double v_cmd = desired_v * (1.0 - std::min(0.8, std::abs(cross_track_error_) * 2.0));
  v_cmd = std::clamp(v_cmd, 0.02, max_linear_vel_);

  // Angular velocity: ω = v · κ  +  PID cross-track correction
  double omega_cmd = v_cmd * curvature + pid_correction;

  // === Step 8: Clamp to robot limits ===
  v_cmd = std::clamp(v_cmd, -max_linear_vel_, max_linear_vel_);
  omega_cmd = std::clamp(omega_cmd, -max_angular_vel_, max_angular_vel_);

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

  // Progress ∈ [0, 1]
  if (!trajectory_.empty()) {
    state.progress = static_cast<double>(nearest_index_) /
                     static_cast<double>(trajectory_.size() - 1);
  }

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
  prev_time_ = -1.0;
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

  // Windowed search around last known index for O(1) amortised
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
  const Pose2D & pose, size_t nearest_idx, double Ld) const
{
  for (size_t i = nearest_idx; i < trajectory_.size(); ++i) {
    double d = std::hypot(pose.x - trajectory_[i].x,
                           pose.y - trajectory_[i].y);
    if (d >= Ld) {
      return i;
    }
  }
  return trajectory_.size() - 1;
}

double PurePursuitController::computeSignedCrossTrackError(
  const Pose2D & pose, size_t idx) const
{
  // Signed error using cross product between trajectory tangent and
  // vector from trajectory point to robot.
  // sign > 0 → robot is to the left of the path.
  if (idx + 1 < trajectory_.size()) {
    double tx = trajectory_[idx + 1].x - trajectory_[idx].x;
    double ty = trajectory_[idx + 1].y - trajectory_[idx].y;
    double ex = pose.x - trajectory_[idx].x;
    double ey = pose.y - trajectory_[idx].y;
    double len = std::hypot(tx, ty);
    if (len > 1e-9) {
      return (tx * ey - ty * ex) / len;   // signed perpendicular distance
    }
  }
  // Fallback: unsigned distance
  return std::hypot(pose.x - trajectory_[idx].x,
                     pose.y - trajectory_[idx].y);
}

}  // namespace smooth_nav_core
