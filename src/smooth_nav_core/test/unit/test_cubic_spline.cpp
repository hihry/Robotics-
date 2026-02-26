/**
 * @file test_cubic_spline.cpp
 * @brief Unit tests for CubicSplineSmoother.
 *
 * Covers: straight line, endpoint matching, arc length monotonicity,
 *         two-point linear interpolation, NaN / too-few-waypoints validation,
 *         factory creation, curvature, heading, duplicate-point handling.
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/path_smoother/cubic_spline_smoother.hpp"
#include "smooth_nav_core/path_smoother/smoother_factory.hpp"
#include "../fixtures/sample_waypoints.hpp"
#include "../fixtures/trajectory_validator.hpp"
#include <cmath>

using namespace smooth_nav_core;

class CubicSplineTest : public ::testing::Test
{
protected:
  CubicSplineSmoother smoother;
};

TEST_F(CubicSplineTest, StraightLineProducesExpectedOutput)
{
  auto wps = test::straightLine();
  auto path = smoother.smooth(wps, 50);

  EXPECT_EQ(path.size(), 50u);

  // All points should lie on y=0 (straight line)
  for (const auto & pt : path) {
    EXPECT_NEAR(pt.y, 0.0, 1e-9);
    EXPECT_GE(pt.x, -1e-9);
    EXPECT_LE(pt.x, 2.0 + 1e-9);
  }
}

TEST_F(CubicSplineTest, EndpointsMatchWaypoints)
{
  auto wps = test::lShape();
  auto path = smoother.smooth(wps, 100);

  EXPECT_NEAR(path.front().x, wps.front().first, 1e-6);
  EXPECT_NEAR(path.front().y, wps.front().second, 1e-6);
  EXPECT_NEAR(path.back().x, wps.back().first, 1e-6);
  EXPECT_NEAR(path.back().y, wps.back().second, 1e-6);
}

TEST_F(CubicSplineTest, ArcLengthsMonotonicallyIncrease)
{
  auto wps = test::sCurve();
  auto path = smoother.smooth(wps, 100);

  EXPECT_TRUE(test::pathArcLengthMonotonic(path));
}

TEST_F(CubicSplineTest, TwoPointLinearInterpolation)
{
  auto wps = test::twoPoints();
  auto path = smoother.smooth(wps, 10);

  EXPECT_EQ(path.size(), 10u);
  // All points on the line y = x
  for (const auto & pt : path) {
    EXPECT_NEAR(pt.x, pt.y, 1e-9);
  }
}

TEST_F(CubicSplineTest, ThrowsOnTooFewWaypoints)
{
  std::vector<std::pair<double, double>> single = {{0.0, 0.0}};
  EXPECT_THROW(smoother.smooth(single, 10), std::invalid_argument);
}

TEST_F(CubicSplineTest, ThrowsOnNaN)
{
  std::vector<std::pair<double, double>> bad = {{0.0, 0.0}, {NAN, 1.0}};
  EXPECT_THROW(smoother.smooth(bad, 10), std::invalid_argument);
}

TEST_F(CubicSplineTest, FactoryCreatesCubicSpline)
{
  auto ptr = SmootherFactory::create("cubic_spline");
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(ptr->name(), "cubic_spline");
}

TEST_F(CubicSplineTest, CurvatureNearZeroForStraightLine)
{
  auto wps = test::straightLine();
  auto path = smoother.smooth(wps, 50);

  // Interior points should have near-zero curvature
  for (size_t i = 2; i + 2 < path.size(); ++i) {
    EXPECT_NEAR(path[i].curvature, 0.0, 1e-3);
  }
}

// ─── New tests ──────────────────────────────────────────────────────────────

TEST_F(CubicSplineTest, HeadingAlongStraightLine)
{
  auto wps = test::straightLine();   // (0,0)→(1,0)→(2,0)
  auto path = smoother.smooth(wps, 50);

  // Heading should be ~0 (pointing in +x direction)
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    EXPECT_NEAR(path[i].theta, 0.0, 0.05);
  }
}

TEST_F(CubicSplineTest, DuplicateWaypointsHandledGracefully)
{
  // Duplicate consecutive points should be auto-removed
  std::vector<std::pair<double, double>> wps = {
    {0, 0}, {0, 0}, {1, 0}, {1, 0}, {2, 0}
  };
  auto path = smoother.smooth(wps, 50);

  EXPECT_EQ(path.size(), 50u);
  // Should still produce a valid straight line
  for (const auto & pt : path) {
    EXPECT_NEAR(pt.y, 0.0, 1e-6);
  }
}

TEST_F(CubicSplineTest, QuarterCircleHasPositiveCurvature)
{
  auto wps = test::quarterCircle();
  auto path = smoother.smooth(wps, 100);

  // Quarter circle should have mostly positive curvature
  int positive_count = 0;
  for (size_t i = 5; i + 5 < path.size(); ++i) {
    if (path[i].curvature > 0.01) positive_count++;
  }
  EXPECT_GT(positive_count, 0);
}
