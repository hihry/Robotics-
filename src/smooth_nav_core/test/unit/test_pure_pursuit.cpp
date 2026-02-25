/**
 * @file test_pure_pursuit.cpp
 * @brief Unit tests for PurePursuitController.
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/controller/pure_pursuit_controller.hpp"
#include "smooth_nav_core/path_smoother/cubic_spline_smoother.hpp"
#include "smooth_nav_core/trajectory_generator/trapezoidal_velocity_generator.hpp"
#include "../fixtures/sample_waypoints.hpp"

using namespace smooth_nav_core;

class PurePursuitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller.setLookAheadDistance(0.3);
    controller.setMaxLinearVelocity(0.18);
    controller.setMaxAngularVelocity(2.0);
    controller.setGoalTolerance(0.05);
    controller.setPIDGains(1.0, 0.0, 0.1);

    // Create a simple straight-line trajectory
    CubicSplineSmoother smoother;
    auto wps = test::straightLine();
    auto path = smoother.smooth(wps, 100);

    TrapezoidalVelocityGenerator gen;
    gen.setMaxVelocity(0.18);
    gen.setMaxAcceleration(0.5);
    gen.setTimeStep(0.05);
    trajectory = gen.generate(path);
  }

  PurePursuitController controller;
  std::vector<TrajectoryPoint> trajectory;
};

TEST_F(PurePursuitTest, ReturnsZeroWhenNoTrajectory)
{
  Pose2D pose(0, 0, 0);
  auto cmd = controller.computeControl(pose, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
}

TEST_F(PurePursuitTest, NotCompleteInitially)
{
  controller.setTrajectory(trajectory);
  EXPECT_FALSE(controller.isComplete());
}

TEST_F(PurePursuitTest, ProducesForwardVelocityOnStraightLine)
{
  controller.setTrajectory(trajectory);
  Pose2D pose(0.0, 0.0, 0.0);
  auto cmd = controller.computeControl(pose, 0.0);

  EXPECT_GT(cmd.linear, 0.0);  // Should move forward
  EXPECT_LE(cmd.linear, 0.18 + 1e-6);
}

TEST_F(PurePursuitTest, CompletesAtGoal)
{
  controller.setTrajectory(trajectory);
  // Place robot right at the goal
  Pose2D at_goal(2.0, 0.0, 0.0);
  auto cmd = controller.computeControl(at_goal, 10.0);

  EXPECT_TRUE(controller.isComplete());
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
}

TEST_F(PurePursuitTest, ResetClearsState)
{
  controller.setTrajectory(trajectory);
  controller.computeControl(Pose2D(0, 0, 0), 0.0);
  controller.reset();

  EXPECT_FALSE(controller.isComplete());
  EXPECT_DOUBLE_EQ(controller.getCrossTrackError(), 0.0);
  EXPECT_DOUBLE_EQ(controller.getHeadingError(), 0.0);
  EXPECT_EQ(controller.getNearestIndex(), 0u);
}

TEST_F(PurePursuitTest, NameReturnsPurePursuit)
{
  EXPECT_EQ(controller.name(), "pure_pursuit");
}

TEST_F(PurePursuitTest, GetStateReturnsValidState)
{
  controller.setTrajectory(trajectory);
  controller.computeControl(Pose2D(0, 0, 0), 0.0);

  auto state = controller.getState();
  EXPECT_GE(state.cross_track_error, 0.0);
  EXPECT_FALSE(state.trajectory_complete);
}
