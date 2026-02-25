/**
 * @file i_controller.hpp
 * @brief Abstract controller interface for trajectory tracking.
 */

#ifndef SMOOTH_NAV_CORE__CONTROLLER__I_CONTROLLER_HPP_
#define SMOOTH_NAV_CORE__CONTROLLER__I_CONTROLLER_HPP_

#include "smooth_nav_core/math/types.hpp"
#include <vector>

namespace smooth_nav_core
{

class IController
{
public:
  virtual ~IController() = default;

  /**
   * @brief Set the trajectory to follow.
   */
  virtual void setTrajectory(const std::vector<TrajectoryPoint> & trajectory) = 0;

  /**
   * @brief Compute velocity command for the robot.
   *
   * @param pose   Current robot pose from odometry.
   * @param time   Current time (seconds).
   * @return Velocity2D (linear, angular) command.
   */
  virtual Velocity2D computeControl(const Pose2D & pose, double time) = 0;

  /**
   * @brief Check if trajectory tracking is complete.
   */
  virtual bool isComplete() const = 0;

  /**
   * @brief Get current controller state for diagnostics.
   */
  virtual ControllerState getState() const = 0;

  /**
   * @brief Reset controller state.
   */
  virtual void reset() = 0;

  virtual std::string name() const = 0;
};

}  // namespace smooth_nav_core

#endif  // SMOOTH_NAV_CORE__CONTROLLER__I_CONTROLLER_HPP_
