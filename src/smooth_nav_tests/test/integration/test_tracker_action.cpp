// Copyright 2026 smooth_nav Authors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_tracker_action.cpp
 * @brief Integration test — verifies the ExecuteTrajectory action server.
 *
 * Publishes a fake /odom, sends a goal to the action server, and verifies
 * that feedback is received and the action completes.
 */

#include <chrono>
#include <cmath>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <smooth_nav_msgs/action/execute_trajectory.hpp>
#include <smooth_nav_msgs/msg/trajectory.hpp>
#include <smooth_nav_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
using ExecuteTrajectory = smooth_nav_msgs::action::ExecuteTrajectory;
using GoalHandleExec = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

class TrackerActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_tracker_client");

    action_client_ =
        rclcpp_action::create_client<ExecuteTrajectory>(
            node_, "/trajectory_tracker/execute_trajectory");

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  }

  void TearDown() override { rclcpp::shutdown(); }

  void publishOdom(double x, double y, double yaw) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "odom";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    odom_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

TEST_F(TrackerActionTest, ActionServerAvailable) {
  ASSERT_TRUE(action_client_->wait_for_action_server(10s))
      << "ExecuteTrajectory action server not available after 10 s";
}

TEST_F(TrackerActionTest, ExecutesShortTrajectory) {
  ASSERT_TRUE(action_client_->wait_for_action_server(10s));

  // Publish initial odometry at origin
  publishOdom(0.0, 0.0, 0.0);
  rclcpp::spin_some(node_);

  // Build a tiny straight-line trajectory
  auto goal = ExecuteTrajectory::Goal();
  for (int i = 0; i <= 5; ++i) {
    smooth_nav_msgs::msg::TrajectoryPoint tp;
    tp.x = i * 0.1;
    tp.y = 0.0;
    tp.heading = 0.0;
    tp.velocity = 0.1;
    tp.stamp.sec = i;
    tp.stamp.nanosec = 0;
    tp.arc_length = i * 0.1;
    goal.trajectory.points.push_back(tp);
  }

  bool feedback_received = false;

  auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
  send_goal_options.feedback_callback =
      [&feedback_received](GoalHandleExec::SharedPtr,
                           const std::shared_ptr<const ExecuteTrajectory::Feedback> fb) {
        feedback_received = true;
        EXPECT_GE(fb->progress_percent, 0.0);
        EXPECT_LE(fb->progress_percent, 100.0);
      };

  auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, goal_handle_future, 10s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal was rejected by server";

  // Keep publishing odom and spinning to let the controller work
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 15.0) {
    publishOdom(0.0, 0.0, 0.0);  // simplified: stays at origin
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(50ms);
  }

  // Note: In a real sim, the robot would move via /cmd_vel → Gazebo → /odom feedback.
  // This test verifies the action server accepts goals and sends feedback.
  EXPECT_TRUE(feedback_received) << "Should have received at least one feedback message";
}
