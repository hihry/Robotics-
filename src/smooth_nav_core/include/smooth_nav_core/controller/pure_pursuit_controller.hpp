/**
 * @file pure_pursuit_controller.hpp
 * @brief Pure Pursuit + PID controller for differential drive trajectory tracking.
 *
 * Reference: R. Craig Coulter, CMU-RI-TR-92-01, 1992
 */

#ifndef SMOOTH_NAV_CORE__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define SMOOTH_NAV_CORE__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include "smooth_nav_core/controller/i_controller.hpp"
#include "smooth_nav_core/controller/pid_controller.hpp"
#include <vector>

namespace smooth_nav_core
{

class PurePursuitController : public IController
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  // Configuration
  void setLookAheadDistance(double distance);
  void setMaxLinearVelocity(double v_max);
  void setMaxAngularVelocity(double omega_max);
  void setGoalTolerance(double tolerance);
  void setPIDGains(double kp, double ki, double kd);

  // IController interface
  void setTrajectory(const std::vector<TrajectoryPoint> & trajectory) override;
  Velocity2D computeControl(const Pose2D & pose, double time) override;
  bool isComplete() const override;
  ControllerState getState() const override;
  void reset() override;
  std::string name() const override { return "pure_pursuit"; }

  // Diagnostics accessors
  double getCrossTrackError() const;
  double getHeadingError() const;
  size_t getNearestIndex() const;

private:
  std::vector<TrajectoryPoint> trajectory_;
  bool trajectory_set_ = false;
  bool trajectory_complete_ = false;

  double look_ahead_distance_ = 0.3;
  double max_linear_vel_ = 0.18;
  double max_angular_vel_ = 2.0;
  double goal_tolerance_ = 0.05;

  PIDController pid_;

  size_t nearest_index_ = 0;
  double cross_track_error_ = 0.0;
  double heading_error_ = 0.0;

  size_t findNearestPoint(const Pose2D & pose) const;
  size_t findLookAheadPoint(const Pose2D & pose, size_t nearest_idx) const;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
