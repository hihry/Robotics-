/**
 * @file test_path_smoother.cpp
 * @brief Unit tests for PathSmoother using Google Test.
 */

#include <gtest/gtest.h>
#include "trajectory_control/path_smoother.hpp"
#include "trajectory_control/types.hpp"
#include <cmath>
#include <vector>

using namespace trajectory_control;

class PathSmootherTest : public ::testing::Test
{
protected:
  PathSmoother smoother;

  std::vector<std::pair<double, double>> makeSquareWaypoints()
  {
    return {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
  }

  std::vector<std::pair<double, double>> makeSCurve()
  {
    return {{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 0}};
  }
};

// ---- Basic functionality tests ----

TEST_F(PathSmootherTest, CubicSplineSmoothProducesMorePoints)
{
  auto waypoints = makeSCurve();
  smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
  smoother.setNumPoints(50);

  auto result = smoother.smooth(waypoints);

  EXPECT_GT(result.size(), waypoints.size());
  EXPECT_EQ(result.size(), static_cast<size_t>(50));
}

TEST_F(PathSmootherTest, GradientDescentSmoothProducesMorePoints)
{
  auto waypoints = makeSCurve();
  smoother.setMethod(SmoothingMethod::GRADIENT_DESCENT);
  smoother.setNumPoints(50);
  smoother.setGradientDescentParams(0.1, 0.3, 1e-6, 5000);

  auto result = smoother.smooth(waypoints);

  EXPECT_GT(result.size(), waypoints.size());
}

TEST_F(PathSmootherTest, CubicSplineStartsAtFirstWaypoint)
{
  auto waypoints = makeSCurve();
  smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
  smoother.setNumPoints(100);

  auto result = smoother.smooth(waypoints);

  EXPECT_NEAR(result.front().x, waypoints.front().first, 1e-6);
  EXPECT_NEAR(result.front().y, waypoints.front().second, 1e-6);
}

TEST_F(PathSmootherTest, CubicSplineEndsAtLastWaypoint)
{
  auto waypoints = makeSCurve();
  smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
  smoother.setNumPoints(100);

  auto result = smoother.smooth(waypoints);

  EXPECT_NEAR(result.back().x, waypoints.back().first, 1e-3);
  EXPECT_NEAR(result.back().y, waypoints.back().second, 1e-3);
}

// ---- Edge case tests ----

TEST_F(PathSmootherTest, TwoPointInput)
{
  std::vector<std::pair<double, double>> waypoints = {{0, 0}, {1, 1}};
  smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
  smoother.setNumPoints(10);

  auto result = smoother.smooth(waypoints);

  EXPECT_GE(result.size(), static_cast<size_t>(2));
}

TEST_F(PathSmootherTest, SmoothedPathIsContinuous)
{
  auto waypoints = makeSquareWaypoints();
  smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
  smoother.setNumPoints(200);

  auto result = smoother.smooth(waypoints);

  // Check that consecutive points are not too far apart
  for (size_t i = 1; i < result.size(); ++i) {
    double dx = result[i].x - result[i - 1].x;
    double dy = result[i].y - result[i - 1].y;
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_LT(dist, 0.5) << "Gap at index " << i;
  }
}

TEST_F(PathSmootherTest, SmoothedPathHasValidCoordinates)
{
  auto waypoints = makeSCurve();
  smoother.setMethod(SmoothingMethod::CUBIC_SPLINE);
  smoother.setNumPoints(100);

  auto result = smoother.smooth(waypoints);

  for (const auto & pt : result) {
    EXPECT_FALSE(std::isnan(pt.x));
    EXPECT_FALSE(std::isnan(pt.y));
    EXPECT_FALSE(std::isinf(pt.x));
    EXPECT_FALSE(std::isinf(pt.y));
  }
}

// ---- Comparison test ----

TEST_F(PathSmootherTest, GradientDescentReducesCurvature)
{
  // Zigzag path
  std::vector<std::pair<double, double>> zigzag = {
    {0, 0}, {1, 1}, {2, -1}, {3, 1}, {4, -1}, {5, 0}};

  smoother.setMethod(SmoothingMethod::GRADIENT_DESCENT);
  smoother.setNumPoints(100);
  smoother.setGradientDescentParams(0.05, 0.5, 1e-6, 10000);

  auto result = smoother.smooth(zigzag);

  // The smoothed path should have smaller max y deviation than zigzag
  double max_y = 0;
  for (const auto & pt : result) {
    max_y = std::max(max_y, std::abs(pt.y));
  }
  // Original zigzag has max |y| = 1.0. Smoothed should be less.
  EXPECT_LT(max_y, 1.0);
}
