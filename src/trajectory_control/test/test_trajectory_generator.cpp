/**
 * @file test_trajectory_generator.cpp
 * @brief Unit tests for TrajectoryGenerator using Google Test.
 */

#include <gtest/gtest.h>
#include "trajectory_control/trajectory_generator.hpp"
#include "trajectory_control/types.hpp"
#include <cmath>
#include <vector>

using namespace trajectory_control;

class TrajectoryGeneratorTest : public ::testing::Test
{
protected:
  TrajectoryGenerator generator;

  std::vector<Pose2D> makeStraightLine(double length, int n)
  {
    std::vector<Pose2D> path;
    for (int i = 0; i < n; ++i) {
      double frac = static_cast<double>(i) / (n - 1);
      path.push_back({frac * length, 0.0, 0.0});
    }
    return path;
  }

  std::vector<Pose2D> makeCircularArc(double radius, double angle_span, int n)
  {
    std::vector<Pose2D> path;
    for (int i = 0; i < n; ++i) {
      double theta = angle_span * i / (n - 1);
      path.push_back({radius * std::cos(theta), radius * std::sin(theta), theta + M_PI / 2});
    }
    return path;
  }
};

// ---- Basic tests ----

TEST_F(TrajectoryGeneratorTest, GeneratesNonEmptyTrajectory)
{
  auto path = makeStraightLine(2.0, 50);
  generator.setMaxVelocity(0.2);
  generator.setMaxAcceleration(0.5);
  generator.setProfileType(VelocityProfileType::TRAPEZOIDAL);

  auto traj = generator.generate(path);

  EXPECT_GT(traj.size(), static_cast<size_t>(0));
}

TEST_F(TrajectoryGeneratorTest, TrapezoidalProfileStartsAtZeroVelocity)
{
  auto path = makeStraightLine(3.0, 100);
  generator.setMaxVelocity(0.2);
  generator.setMaxAcceleration(0.5);
  generator.setProfileType(VelocityProfileType::TRAPEZOIDAL);

  auto traj = generator.generate(path);

  ASSERT_GT(traj.size(), static_cast<size_t>(0));
  EXPECT_NEAR(traj.front().v, 0.0, 1e-3);
}

TEST_F(TrajectoryGeneratorTest, TrapezoidalProfileEndsAtZeroVelocity)
{
  auto path = makeStraightLine(3.0, 100);
  generator.setMaxVelocity(0.2);
  generator.setMaxAcceleration(0.5);
  generator.setProfileType(VelocityProfileType::TRAPEZOIDAL);

  auto traj = generator.generate(path);

  ASSERT_GT(traj.size(), static_cast<size_t>(0));
  EXPECT_NEAR(traj.back().v, 0.0, 1e-2);
}

TEST_F(TrajectoryGeneratorTest, VelocityNeverExceedsMax)
{
  auto path = makeStraightLine(5.0, 200);
  double max_v = 0.18;
  generator.setMaxVelocity(max_v);
  generator.setMaxAcceleration(0.5);
  generator.setProfileType(VelocityProfileType::TRAPEZOIDAL);

  auto traj = generator.generate(path);

  for (const auto & pt : traj) {
    EXPECT_LE(pt.v, max_v + 1e-6) << "at t=" << pt.timestamp;
  }
}

TEST_F(TrajectoryGeneratorTest, ConstantProfileMaintainsVelocity)
{
  auto path = makeStraightLine(3.0, 100);
  double max_v = 0.15;
  generator.setMaxVelocity(max_v);
  generator.setProfileType(VelocityProfileType::CONSTANT);

  auto traj = generator.generate(path);

  // All points should have velocity ≈ max_v
  for (const auto & pt : traj) {
    EXPECT_NEAR(pt.v, max_v, 1e-3);
  }
}

TEST_F(TrajectoryGeneratorTest, TimestampsAreMonotonicallyIncreasing)
{
  auto path = makeStraightLine(2.0, 50);
  generator.setMaxVelocity(0.2);
  generator.setMaxAcceleration(0.5);
  generator.setProfileType(VelocityProfileType::TRAPEZOIDAL);

  auto traj = generator.generate(path);

  for (size_t i = 1; i < traj.size(); ++i) {
    EXPECT_GT(traj[i].timestamp, traj[i - 1].timestamp)
      << "Timestamps not monotonic at index " << i;
  }
}

TEST_F(TrajectoryGeneratorTest, ArcLengthIsMonotonicallyIncreasing)
{
  auto path = makeStraightLine(2.0, 50);
  generator.setMaxVelocity(0.2);
  generator.setMaxAcceleration(0.5);

  auto traj = generator.generate(path);

  for (size_t i = 1; i < traj.size(); ++i) {
    EXPECT_GE(traj[i].arc_length, traj[i - 1].arc_length - 1e-9)
      << "Arc length not monotonic at index " << i;
  }
}

TEST_F(TrajectoryGeneratorTest, TrajectoryPointsHaveValidCoordinates)
{
  auto path = makeCircularArc(1.0, M_PI, 50);
  generator.setMaxVelocity(0.2);
  generator.setMaxAcceleration(0.5);

  auto traj = generator.generate(path);

  for (const auto & pt : traj) {
    EXPECT_FALSE(std::isnan(pt.x));
    EXPECT_FALSE(std::isnan(pt.y));
    EXPECT_FALSE(std::isnan(pt.v));
    EXPECT_FALSE(std::isnan(pt.omega));
    EXPECT_FALSE(std::isinf(pt.x));
    EXPECT_FALSE(std::isinf(pt.y));
  }
}

TEST_F(TrajectoryGeneratorTest, ComputesTotalArcLength)
{
  auto path = makeStraightLine(3.0, 100);
  double len = generator.computeTotalArcLength(path);
  EXPECT_NEAR(len, 3.0, 0.1);
}
