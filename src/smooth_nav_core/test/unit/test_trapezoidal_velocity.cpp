/**
 * @file test_trapezoidal_velocity.cpp
 * @brief Unit tests for TrapezoidalVelocityGenerator and ConstantVelocityGenerator.
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/trajectory_generator/trapezoidal_velocity_generator.hpp"
#include "smooth_nav_core/trajectory_generator/constant_velocity_generator.hpp"
#include "smooth_nav_core/path_smoother/cubic_spline_smoother.hpp"
#include "../fixtures/sample_waypoints.hpp"
#include "../fixtures/trajectory_validator.hpp"

using namespace smooth_nav_core;

class TrapezoidalVelocityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gen.setMaxVelocity(0.18);
    gen.setMaxAcceleration(0.5);
    gen.setTimeStep(0.05);
  }

  TrapezoidalVelocityGenerator gen;
  CubicSplineSmoother smoother;
};

TEST_F(TrapezoidalVelocityTest, GeneratesStraightLineTrajectory)
{
  auto wps = test::straightLine();
  auto path = smoother.smooth(wps, 100);
  auto traj = gen.generate(path);

  EXPECT_GT(traj.size(), 2u);
  EXPECT_TRUE(test::isTimeMonotonic(traj));
  EXPECT_TRUE(test::isArcLengthMonotonic(traj));
  EXPECT_TRUE(test::endsWithZeroVelocity(traj));
}

TEST_F(TrapezoidalVelocityTest, VelocitiesWithinBounds)
{
  auto wps = test::lShape();
  auto path = smoother.smooth(wps, 100);
  auto traj = gen.generate(path);

  // All velocities should be <= v_max
  for (const auto & pt : traj) {
    EXPECT_LE(pt.v, 0.18 + 1e-6);
    EXPECT_GE(pt.v, 0.0 - 1e-6);
  }
}

TEST_F(TrapezoidalVelocityTest, ThrowsOnSinglePoint)
{
  std::vector<PathPoint> single = {PathPoint(0, 0)};
  EXPECT_THROW(gen.generate(single), std::invalid_argument);
}

TEST_F(TrapezoidalVelocityTest, ThrowsOnInvalidParams)
{
  EXPECT_THROW(gen.setMaxVelocity(-1.0), std::invalid_argument);
  EXPECT_THROW(gen.setMaxAcceleration(0.0), std::invalid_argument);
  EXPECT_THROW(gen.setTimeStep(-0.01), std::invalid_argument);
}

// ─── ConstantVelocityGenerator Tests ────────────────────────────────────────

class ConstantVelocityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gen.setMaxVelocity(0.18);
    gen.setTimeStep(0.05);
  }

  ConstantVelocityGenerator gen;
  CubicSplineSmoother smoother;
};

TEST_F(ConstantVelocityTest, GeneratesTrajectory)
{
  auto wps = test::straightLine();
  auto path = smoother.smooth(wps, 100);
  auto traj = gen.generate(path);

  EXPECT_GT(traj.size(), 2u);
  EXPECT_TRUE(test::isTimeMonotonic(traj));
  EXPECT_TRUE(test::endsWithZeroVelocity(traj));
}

TEST_F(ConstantVelocityTest, InteriorPointsHaveConstantVelocity)
{
  auto wps = test::straightLine();
  auto path = smoother.smooth(wps, 100);
  auto traj = gen.generate(path);

  // All non-last points should have v == v_max
  for (size_t i = 0; i + 1 < traj.size(); ++i) {
    EXPECT_NEAR(traj[i].v, 0.18, 1e-6);
  }
}
