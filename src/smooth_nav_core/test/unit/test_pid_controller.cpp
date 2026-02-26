/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for the standalone PIDController utility.
 *
 * Tests cover:
 *   - Proportional, integral, derivative channels (backward-compat API)
 *   - dt-aware compute()
 *   - Integral anti-windup
 *   - Derivative low-pass filter
 *   - Output clamping
 *   - Reset behaviour
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/controller/pid_controller.hpp"
#include <cmath>

using namespace smooth_nav_core;

// ─── Backward-compatible (computeSimple) ────────────────────────────────────

TEST(PIDControllerTest, ProportionalOutput)
{
  PIDController pid(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(5.0), 5.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(3.0), 3.0);
}

TEST(PIDControllerTest, IntegralAccumulates)
{
  PIDController pid(0.0, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(1.0), 1.0);   // integral = 1
  EXPECT_DOUBLE_EQ(pid.computeSimple(1.0), 2.0);   // integral = 2
  EXPECT_DOUBLE_EQ(pid.computeSimple(1.0), 3.0);   // integral = 3
}

TEST(PIDControllerTest, DerivativeResponds)
{
  PIDController pid(0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(0.0), 0.0);   // deriv = 0 - 0 = 0
  EXPECT_DOUBLE_EQ(pid.computeSimple(5.0), 5.0);   // deriv = 5 - 0 = 5
  EXPECT_DOUBLE_EQ(pid.computeSimple(5.0), 0.0);   // deriv = 5 - 5 = 0
}

TEST(PIDControllerTest, ResetClearsHistory)
{
  PIDController pid(1.0, 1.0, 1.0);
  pid.computeSimple(10.0);
  pid.computeSimple(20.0);
  pid.reset();

  // After reset, same as fresh controller
  double out = pid.computeSimple(1.0);
  // P = 1*1 = 1, I = 1*1 = 1 (integral starts fresh), D = 1*(1-0) = 1
  EXPECT_DOUBLE_EQ(out, 3.0);
}

TEST(PIDControllerTest, ZeroErrorZeroOutput)
{
  PIDController pid(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(0.0), 0.0);
}

TEST(PIDControllerTest, SetGainsUpdates)
{
  PIDController pid;
  pid.setGains(2.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(3.0), 6.0);
}

TEST(PIDControllerTest, NegativeError)
{
  PIDController pid(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.computeSimple(-5.0), -5.0);
}

// ─── dt-aware compute() ─────────────────────────────────────────────────────

TEST(PIDControllerTest, ComputeWithDtScalesIntegral)
{
  PIDController pid(0.0, 1.0, 0.0);
  pid.setDerivativeFilter(0.0);  // disable filter for clean test

  // dt = 0.5 → integral += error * 0.5
  double out = pid.compute(2.0, 0.5);
  // integral = 2.0 * 0.5 = 1.0 → i_term = 1.0
  EXPECT_NEAR(out, 1.0, 1e-12);
}

TEST(PIDControllerTest, ComputeWithDtScalesDerivative)
{
  PIDController pid(0.0, 0.0, 1.0);
  pid.setDerivativeFilter(0.0);  // raw derivative only

  pid.compute(0.0, 0.1);            // prev_error = 0
  double out = pid.compute(1.0, 0.1); // raw_deriv = (1 - 0) / 0.1 = 10
  EXPECT_NEAR(out, 10.0, 1e-12);
}

// ─── Derivative low-pass filter ─────────────────────────────────────────────

TEST(PIDControllerTest, DerivativeFilterSmooths)
{
  PIDController pid(0.0, 0.0, 1.0);
  pid.setDerivativeFilter(0.5);  // heavy smoothing

  pid.compute(0.0, 1.0);                // prev = 0, filtered_d = 0
  double out = pid.compute(10.0, 1.0);  // raw_d = 10
  // filtered = 0.5 * 0 + 0.5 * 10 = 5  → d_term = 5
  EXPECT_NEAR(out, 5.0, 1e-12);

  double out2 = pid.compute(10.0, 1.0); // raw_d = 0
  // filtered = 0.5 * 5 + 0.5 * 0 = 2.5 → d_term = 2.5
  EXPECT_NEAR(out2, 2.5, 1e-12);
}

// ─── Output clamping ────────────────────────────────────────────────────────

TEST(PIDControllerTest, OutputClampsSymmetric)
{
  PIDController pid(10.0, 0.0, 0.0);
  pid.setOutputLimits(5.0);
  pid.setDerivativeFilter(0.0);

  EXPECT_NEAR(pid.compute(100.0, 1.0), 5.0, 1e-12);
  pid.reset();
  EXPECT_NEAR(pid.compute(-100.0, 1.0), -5.0, 1e-12);
}

TEST(PIDControllerTest, OutputClampsAsymmetric)
{
  PIDController pid(10.0, 0.0, 0.0);
  pid.setOutputLimits(-2.0, 8.0);
  pid.setDerivativeFilter(0.0);

  EXPECT_NEAR(pid.compute(100.0, 1.0), 8.0, 1e-12);
  pid.reset();
  EXPECT_NEAR(pid.compute(-100.0, 1.0), -2.0, 1e-12);
}

// ─── Integral anti-windup ───────────────────────────────────────────────────

TEST(PIDControllerTest, IntegralLimitPreventsWindup)
{
  PIDController pid(0.0, 1.0, 0.0);
  pid.setIntegralLimit(5.0);
  pid.setDerivativeFilter(0.0);

  // Push integral well past limit
  for (int i = 0; i < 100; ++i) {
    pid.compute(10.0, 1.0);
  }
  // i_term should be clamped at 5.0
  EXPECT_NEAR(pid.getIntegral(), 5.0, 1e-9);
}

// ─── Accessor ───────────────────────────────────────────────────────────────

TEST(PIDControllerTest, GetGains)
{
  PIDController pid(1.5, 2.5, 3.5);
  EXPECT_DOUBLE_EQ(pid.getProportionalGain(), 1.5);
  EXPECT_DOUBLE_EQ(pid.getIntegralGain(), 2.5);
  EXPECT_DOUBLE_EQ(pid.getDerivativeGain(), 3.5);
}
