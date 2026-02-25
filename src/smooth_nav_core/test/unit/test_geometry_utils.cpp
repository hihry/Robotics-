/**
 * @file test_geometry_utils.cpp
 * @brief Unit tests for geometry utility functions.
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/math/geometry_utils.hpp"
#include <cmath>

using namespace smooth_nav_core;

TEST(GeometryUtilsTest, DistSamePoint)
{
  EXPECT_DOUBLE_EQ(dist(0, 0, 0, 0), 0.0);
}

TEST(GeometryUtilsTest, DistKnownValues)
{
  EXPECT_NEAR(dist(0, 0, 3, 4), 5.0, 1e-12);
  EXPECT_NEAR(dist(1, 1, 4, 5), 5.0, 1e-12);
}

TEST(GeometryUtilsTest, NormalizeAngleInRange)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(normalizeAngle(-M_PI), -M_PI, 1e-12);
}

TEST(GeometryUtilsTest, NormalizeAngleWraps)
{
  EXPECT_NEAR(normalizeAngle(3 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(normalizeAngle(-3 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(normalizeAngle(2 * M_PI), 0.0, 1e-12);
}

TEST(GeometryUtilsTest, LerpEndpoints)
{
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 1.0), 10.0);
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
}

TEST(GeometryUtilsTest, ComputeArcLengths)
{
  std::vector<std::pair<double, double>> pts = {
    {0, 0}, {1, 0}, {1, 1}
  };
  auto arc = computeArcLengths(pts);

  EXPECT_EQ(arc.size(), 3u);
  EXPECT_DOUBLE_EQ(arc[0], 0.0);
  EXPECT_NEAR(arc[1], 1.0, 1e-12);
  EXPECT_NEAR(arc[2], 2.0, 1e-12);
}

TEST(GeometryUtilsTest, ComputeArcLengthsSinglePoint)
{
  std::vector<std::pair<double, double>> pts = {{5, 3}};
  auto arc = computeArcLengths(pts);
  EXPECT_EQ(arc.size(), 1u);
  EXPECT_DOUBLE_EQ(arc[0], 0.0);
}
