/**
 * @file test_geometry_utils.cpp
 * @brief Unit tests for geometry utility functions.
 *
 * Covers: dist, normalizeAngle, angleDiff, lerp, computeArcLengths,
 *         dot2D, cross2D, closestPointOnSegment, signedDistToLine,
 *         pointToSegmentDistance, removeDuplicates.
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/math/geometry_utils.hpp"
#include <cmath>

using namespace smooth_nav_core;

// ─── dist ───────────────────────────────────────────────────────────────────

TEST(GeometryUtilsTest, DistSamePoint)
{
  EXPECT_DOUBLE_EQ(dist(0, 0, 0, 0), 0.0);
}

TEST(GeometryUtilsTest, DistKnownValues)
{
  EXPECT_NEAR(dist(0, 0, 3, 4), 5.0, 1e-12);
  EXPECT_NEAR(dist(1, 1, 4, 5), 5.0, 1e-12);
}

// ─── normalizeAngle ─────────────────────────────────────────────────────────

TEST(GeometryUtilsTest, NormalizeAngleInRange)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(normalizeAngle(-M_PI), -M_PI, 1e-12);
}

TEST(GeometryUtilsTest, NormalizeAngleWraps)
{
  // std::remainder(3π, 2π) = -π  (both ±π represent the same angle)
  EXPECT_NEAR(std::abs(normalizeAngle(3 * M_PI)), M_PI, 1e-12);
  EXPECT_NEAR(std::abs(normalizeAngle(-3 * M_PI)), M_PI, 1e-12);
  EXPECT_NEAR(normalizeAngle(2 * M_PI), 0.0, 1e-12);
}

// ─── angleDiff ──────────────────────────────────────────────────────────────

TEST(GeometryUtilsTest, AngleDiffZero)
{
  EXPECT_NEAR(angleDiff(1.0, 1.0), 0.0, 1e-12);
}

TEST(GeometryUtilsTest, AngleDiffWrapsCorrectly)
{
  // Going from -π+0.1 to π-0.1  should be a short negative step
  double d = angleDiff(-M_PI + 0.1, M_PI - 0.1);
  EXPECT_NEAR(d, -0.2, 1e-12);
}

// ─── lerp ───────────────────────────────────────────────────────────────────

TEST(GeometryUtilsTest, LerpEndpoints)
{
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 1.0), 10.0);
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
}

// ─── computeArcLengths ─────────────────────────────────────────────────────

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

// ─── dot2D / cross2D ───────────────────────────────────────────────────────

TEST(GeometryUtilsTest, Dot2DOrthogonal)
{
  EXPECT_NEAR(dot2D(1, 0, 0, 1), 0.0, 1e-12);
}

TEST(GeometryUtilsTest, Dot2DParallel)
{
  EXPECT_NEAR(dot2D(3, 4, 3, 4), 25.0, 1e-12);
}

TEST(GeometryUtilsTest, Cross2DOrthogonal)
{
  EXPECT_NEAR(cross2D(1, 0, 0, 1), 1.0, 1e-12);
  EXPECT_NEAR(cross2D(0, 1, 1, 0), -1.0, 1e-12);
}

// ─── closestPointOnSegment ──────────────────────────────────────────────────

TEST(GeometryUtilsTest, ClosestPointOnSegmentMidpoint)
{
  // Segment from (0,0) to (10,0), query at (5,3)
  double t;
  auto [cx, cy] = closestPointOnSegment(0, 0, 10, 0, 5, 3, t);
  EXPECT_NEAR(cx, 5.0, 1e-12);
  EXPECT_NEAR(cy, 0.0, 1e-12);
  EXPECT_NEAR(t, 0.5, 1e-12);
}

TEST(GeometryUtilsTest, ClosestPointOnSegmentClampsToStart)
{
  double t;
  auto [cx, cy] = closestPointOnSegment(0, 0, 10, 0, -5, 1, t);
  EXPECT_NEAR(cx, 0.0, 1e-12);
  EXPECT_NEAR(cy, 0.0, 1e-12);
  EXPECT_NEAR(t, 0.0, 1e-12);
}

// ─── signedDistToLine ───────────────────────────────────────────────────────

TEST(GeometryUtilsTest, SignedDistToLinePositive)
{
  // Point above the x-axis line through (0,0)→(1,0) → positive
  double d = signedDistToLine(0, 0, 1, 0, 0.5, 1.0);
  EXPECT_NEAR(d, 1.0, 1e-12);
}

TEST(GeometryUtilsTest, SignedDistToLineNegative)
{
  // Point below the x-axis
  double d = signedDistToLine(0, 0, 1, 0, 0.5, -2.0);
  EXPECT_NEAR(d, -2.0, 1e-12);
}

// ─── pointToSegmentDistance ─────────────────────────────────────────────────

TEST(GeometryUtilsTest, PointToSegmentDistancePerp)
{
  // Perpendicular distance from (5,3) to segment (0,0)→(10,0) = 3
  EXPECT_NEAR(pointToSegmentDistance(0, 0, 10, 0, 5, 3), 3.0, 1e-12);
}

// ─── removeDuplicates ───────────────────────────────────────────────────────

TEST(GeometryUtilsTest, RemoveDuplicatesNoDupes)
{
  std::vector<std::pair<double,double>> pts = {{0,0},{1,0},{2,0}};
  auto clean = removeDuplicates(pts);
  EXPECT_EQ(clean.size(), 3u);
}

TEST(GeometryUtilsTest, RemoveDuplicatesRemovesConsecutive)
{
  std::vector<std::pair<double,double>> pts = {{0,0},{0,0},{1,0},{1,0},{2,0}};
  auto clean = removeDuplicates(pts);
  EXPECT_EQ(clean.size(), 3u);
  EXPECT_NEAR(clean[0].first, 0.0, 1e-12);
  EXPECT_NEAR(clean[1].first, 1.0, 1e-12);
  EXPECT_NEAR(clean[2].first, 2.0, 1e-12);
}
