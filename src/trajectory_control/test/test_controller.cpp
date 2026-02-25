/**
 * @file test_controller.cpp
 * @brief Unit tests for TrajectoryController using Google Test.
 */

#include <gtest/gtest.h>
#include "trajectory_control/trajectory_controller.hpp"
#include "trajectory_control/types.hpp"
#include <cmath>
#include <vector>

using namespace trajectory_control;

class ControllerTest : public ::testing::Test
{
protected:
  TrajectoryController controller;

  std::vector<TrajectoryPoint> makeStraightTrajectory()
  {
    std::vector<TrajectoryPoint> traj;
    for (int i = 0; i <= 100; ++i) {
      TrajectoryPoint tp;
      tp.x = i * 0.05;       // 0..5 m
      tp.y = 0.0;
      tp.theta = 0.0;
      tp.v = 0.2;
      tp.omega = 0.0;
      tp.curvature = 0.0;
      tp.timestamp = i * 0.25;
      tp.arc_length = i * 0.05;
      traj.push_back(tp);
    }
    return traj;
  }

  void setupController()
  {
    controller.setLookAheadDistance(0.3);
    controller.setMaxLinearVelocity(0.2);
    controller.setMaxAngularVelocity(2.0);
    controller.setGoalTolerance(0.1);
    controller.setPIDGains(1.0, 0.0, 0.1);
    controller.setTrajectory(makeStraightTrajectory());
  }
};

// ---- Basic tests ----

TEST_F(ControllerTest, ComputeControlReturnsValidOutput)
{
  setupController();
  Pose2D pose{0.0, 0.0, 0.0};

  auto [v, omega] = controller.computeControl(pose, 0.0);

  EXPECT_FALSE(std::isnan(v));
  EXPECT_FALSE(std::isnan(omega));
  EXPECT_GE(v, 0.0);
}

TEST_F(ControllerTest, ForwardVelocityOnStraightPath)
{
  setupController();
  Pose2D pose{0.0, 0.0, 0.0};  // At start, facing along path

  auto [v, omega] = controller.computeControl(pose, 0.0);

  EXPECT_GT(v, 0.0);
  // Should have minimal angular velocity when aligned with path
  EXPECT_LT(std::abs(omega), 0.5);
}

TEST_F(ControllerTest, AngularCorrectionWhenOffPath)
{
  setupController();
  // Robot is offset from path by 0.3m in y, facing along x
  Pose2D pose{1.0, 0.3, 0.0};

  auto [v, omega] = controller.computeControl(pose, 1.0);

  // Should steer back toward the path (negative omega to correct positive y offset)
  // The sign depends on pure pursuit geometry
  EXPECT_NE(omega, 0.0);
}

TEST_F(ControllerTest, TrajectoryCompleteAtEnd)
{
  setupController();
  // Place robot very close to the final point
  Pose2D pose{5.0, 0.0, 0.0};

  // call computeControl to update state
  controller.computeControl(pose, 25.0);

  EXPECT_TRUE(controller.isTrajectoryComplete());
}

TEST_F(ControllerTest, TrajectoryNotCompleteAtStart)
{
  setupController();
  Pose2D pose{0.0, 0.0, 0.0};

  controller.computeControl(pose, 0.0);

  EXPECT_FALSE(controller.isTrajectoryComplete());
}

TEST_F(ControllerTest, VelocityRespectsBounds)
{
  setupController();

  // Test from several poses
  std::vector<Pose2D> poses = {
    {0.0, 0.0, 0.0},
    {1.0, 0.5, 0.5},
    {2.0, -0.3, -0.2},
    {3.0, 0.0, M_PI / 4},
  };

  for (const auto & pose : poses) {
    auto [v, omega] = controller.computeControl(pose, 0.0);

    EXPECT_LE(v, 0.2 + 1e-6);
    EXPECT_GE(v, 0.0);
    EXPECT_LE(std::abs(omega), 2.0 + 1e-6);
  }
}

TEST_F(ControllerTest, CrossTrackErrorUpdates)
{
  setupController();

  Pose2D pose1{0.0, 0.0, 0.0};
  controller.computeControl(pose1, 0.0);
  double err1 = controller.getCrossTrackError();

  Pose2D pose2{1.0, 0.5, 0.0};
  controller.computeControl(pose2, 1.0);
  double err2 = controller.getCrossTrackError();

  // Error should be larger when further from path
  EXPECT_GT(err2, err1);
}

TEST_F(ControllerTest, HeadingErrorReported)
{
  setupController();

  Pose2D pose{1.0, 0.0, M_PI / 4};  // 45° offset from path heading
  controller.computeControl(pose, 1.0);
  double heading_err = controller.getHeadingError();

  EXPECT_GT(std::abs(heading_err), 0.1);
}
