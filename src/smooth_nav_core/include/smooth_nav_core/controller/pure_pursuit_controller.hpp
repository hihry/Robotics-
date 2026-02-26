/**
 * @file pure_pursuit_controller.hpp
 * @brief Pure Pursuit + PID controller for differential drive trajectory tracking.
 *
 * Upgrades over basic Pure Pursuit:
 *   1. **Adaptive look-ahead** — L_d = L_d_min + k_ld · v.  Scales with speed so
 *      the controller is tight at low speeds and stable at high speeds.
 *   2. **Signed cross-track error** — cross-product sign tells which side of the
 *      path the robot is on, enabling proper PID correction direction.
 *   3. **Velocity feedforward** — uses the trajectory's reference velocity (from
 *      trapezoidal/constant generator) rather than a fixed speed.
 *   4. **Goal deceleration** — linearly reduces speed when close to the goal.
 *   5. **Progress tracking** — populates ControllerState::progress ∈ [0, 1].
 *   6. **Circle-line look-ahead** — interpolates between trajectory points for
 *      a more accurate look-ahead on sparse paths.
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

  // ─── Configuration ──────────────────────────────────────────
  void setLookAheadDistance(double distance);
  void setMaxLinearVelocity(double v_max);
  void setMaxAngularVelocity(double omega_max);
  void setGoalTolerance(double tolerance);
  void setPIDGains(double kp, double ki, double kd);

  /** @brief Set adaptive look-ahead gain.  L_d = L_d_base + k * |v|.
   *  Set k = 0 for fixed look-ahead (classic behaviour). */
  void setAdaptiveLookAheadGain(double k);

  /** @brief Set deceleration distance before goal (meters).
   *  Inside this radius the speed is linearly ramped to min_speed. */
  void setGoalDecelerationRadius(double radius);

  // ─── IController interface ──────────────────────────────────
  void setTrajectory(const std::vector<TrajectoryPoint> & trajectory) override;
  Velocity2D computeControl(const Pose2D & pose, double time) override;
  bool isComplete() const override;
  ControllerState getState() const override;
  void reset() override;
  std::string name() const override { return "pure_pursuit"; }

  // ─── Diagnostics ────────────────────────────────────────────
  double getCrossTrackError() const;
  double getHeadingError() const;
  size_t getNearestIndex() const;

private:
  std::vector<TrajectoryPoint> trajectory_;
  bool trajectory_set_ = false;
  bool trajectory_complete_ = false;

  // Parameters
  double look_ahead_distance_ = 0.3;
  double adaptive_k_ = 0.0;            // L_d gain w.r.t. speed
  double max_linear_vel_ = 0.18;
  double max_angular_vel_ = 2.0;
  double goal_tolerance_ = 0.05;
  double goal_decel_radius_ = 0.3;     // deceleration zone near goal

  // PID for cross-track correction
  PIDController pid_;

  // Runtime state
  size_t nearest_index_ = 0;
  double cross_track_error_ = 0.0;
  double heading_error_ = 0.0;
  double prev_time_ = -1.0;

  // Private helpers
  size_t findNearestPoint(const Pose2D & pose) const;
  size_t findLookAheadPoint(const Pose2D & pose, size_t nearest_idx,
                            double Ld) const;
  double computeSignedCrossTrackError(const Pose2D & pose, size_t idx) const;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
