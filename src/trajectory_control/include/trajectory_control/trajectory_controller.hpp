/**
 * @file trajectory_controller.hpp
 * @brief Pure Pursuit + PID trajectory tracking controller for differential drive robots.
 *
 * Implements the Pure Pursuit path tracking algorithm adapted for a differential
 * drive robot (TurtleBot3). Outputs (v, omega) velocity commands.
 *
 * Pure Pursuit Algorithm:
 *   1. Find the nearest point on the trajectory to the robot
 *   2. Find a look-ahead point at distance L_d ahead on the trajectory
 *   3. Compute the curvature: κ = 2*sin(α) / L_d
 *   4. Compute angular velocity: ω = v * κ
 *   5. Clamp to robot velocity limits
 *
 * References:
 *   - R. Craig Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm"
 *     CMU-RI-TR-92-01, 1992
 *   - PythonRobotics: PathTracking/pure_pursuit/pure_pursuit.py
 */

#ifndef TRAJECTORY_CONTROL__TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROL__TRAJECTORY_CONTROLLER_HPP_

#include "trajectory_control/types.hpp"
#include <vector>
#include <utility>

namespace trajectory_control
{

/**
 * @class TrajectoryController
 * @brief Pure Pursuit controller for differential drive trajectory tracking.
 *
 * Usage:
 * @code
 *   TrajectoryController ctrl;
 *   ctrl.setLookAheadDistance(0.3);
 *   ctrl.setTrajectory(trajectory);
 *   auto [v, omega] = ctrl.computeControl(current_pose, current_time);
 * @endcode
 */
class TrajectoryController
{
public:
  TrajectoryController();
  ~TrajectoryController() = default;

  // ---- Configuration ----

  /// Set look-ahead distance for Pure Pursuit (meters)
  void setLookAheadDistance(double distance);

  /// Set maximum allowed linear velocity (m/s)
  void setMaxLinearVelocity(double v_max);

  /// Set maximum allowed angular velocity (rad/s)
  void setMaxAngularVelocity(double omega_max);

  /// Set goal tolerance — distance to final point to consider trajectory complete (m)
  void setGoalTolerance(double tolerance);

  /// Set PID gains for velocity tracking
  void setPIDGains(double kp, double ki, double kd);

  /// Load the trajectory to follow
  void setTrajectory(const std::vector<TrajectoryPoint> & trajectory);

  // ---- Core API ----

  /**
   * @brief Compute velocity command (v, omega) based on current robot state.
   *
   * @param current_pose Robot's current pose (x, y, theta) from odometry.
   * @param current_time Current simulation time (seconds).
   * @return std::pair<double, double> (linear_velocity, angular_velocity)
   *         Returns (0, 0) if trajectory is complete or not set.
   */
  std::pair<double, double> computeControl(
    const Pose2D & current_pose,
    double current_time);

  // ---- Status Queries ----

  /// Check if the robot has reached the end of the trajectory
  bool isTrajectoryComplete() const;

  /// Get the current cross-track error (perpendicular distance to nearest point)
  double getCrossTrackError() const;

  /// Get the current heading error (radians)
  double getHeadingError() const;

  /// Get the index of the nearest trajectory point
  size_t getNearestIndex() const;

  /// Reset the controller state (for restarting a new trajectory)
  void reset();

private:
  // Trajectory
  std::vector<TrajectoryPoint> trajectory_;
  bool trajectory_set_ = false;
  bool trajectory_complete_ = false;

  // Parameters
  double look_ahead_distance_ = 0.3;   ///< L_d in Pure Pursuit (m)
  double max_linear_vel_ = 0.18;       ///< TurtleBot3 Burger limit
  double max_angular_vel_ = 2.0;       ///< TurtleBot3 Burger limit (rad/s)
  double goal_tolerance_ = 0.05;       ///< 5cm to consider goal reached

  // PID gains for velocity error correction
  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.1;
  double integral_error_ = 0.0;
  double prev_error_ = 0.0;

  // Tracking state
  size_t nearest_index_ = 0;
  double cross_track_error_ = 0.0;
  double heading_error_ = 0.0;

  /**
   * @brief Find the nearest point on the trajectory to the robot.
   * @param pose Current robot pose.
   * @return Index of nearest trajectory point.
   */
  size_t findNearestPoint(const Pose2D & pose) const;

  /**
   * @brief Find the look-ahead point on the trajectory.
   *
   * Searches forward from nearest_index to find the first point
   * at distance >= look_ahead_distance from the robot.
   *
   * @param pose Current robot pose.
   * @param nearest_idx Index of the nearest trajectory point.
   * @return Index of the look-ahead point.
   */
  size_t findLookAheadPoint(const Pose2D & pose, size_t nearest_idx) const;

  /**
   * @brief Normalize angle to [-pi, pi].
   */
  static double normalizeAngle(double angle);
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TRAJECTORY_CONTROLLER_HPP_
