/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for the standalone PIDController utility.
 */

#include <gtest/gtest.h>
#include "smooth_nav_core/controller/pid_controller.hpp"
#include <cmath>

using namespace smooth_nav_core;

TEST(PIDControllerTest, ProportionalOutput)
{
  PIDController pid(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.compute(5.0), 5.0);
  EXPECT_DOUBLE_EQ(pid.compute(3.0), 3.0);
}

TEST(PIDControllerTest, IntegralAccumulates)
{
  PIDController pid(0.0, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.compute(1.0), 1.0);   // integral = 1
  EXPECT_DOUBLE_EQ(pid.compute(1.0), 2.0);   // integral = 2
  EXPECT_DOUBLE_EQ(pid.compute(1.0), 3.0);   // integral = 3
}

TEST(PIDControllerTest, DerivativeResponds)
{
  PIDController pid(0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(pid.compute(0.0), 0.0);   // deriv = 0 - 0 = 0
  EXPECT_DOUBLE_EQ(pid.compute(5.0), 5.0);   // deriv = 5 - 0 = 5
  EXPECT_DOUBLE_EQ(pid.compute(5.0), 0.0);   // deriv = 5 - 5 = 0
}

TEST(PIDControllerTest, ResetClearsHistory)
{
  PIDController pid(1.0, 1.0, 1.0);
  pid.compute(10.0);
  pid.compute(20.0);
  pid.reset();

  // After reset, same as fresh controller
  double out = pid.compute(1.0);
  // P = 1*1 = 1, I = 1*1 = 1 (integral starts fresh), D = 1*(1-0) = 1
  EXPECT_DOUBLE_EQ(out, 3.0);
}

TEST(PIDControllerTest, ZeroErrorZeroOutput)
{
  PIDController pid(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.compute(0.0), 0.0);
}

TEST(PIDControllerTest, SetGainsUpdates)
{
  PIDController pid;
  pid.setGains(2.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.compute(3.0), 6.0);
}

TEST(PIDControllerTest, NegativeError)
{
  PIDController pid(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pid.compute(-5.0), -5.0);
}
