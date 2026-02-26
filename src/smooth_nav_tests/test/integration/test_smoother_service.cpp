// Copyright 2026 smooth_nav Authors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_smoother_service.cpp
 * @brief Integration test — verifies the /path_smoother/smooth_path service.
 *
 * Spins up a rclcpp node, calls the SmoothPath service, and checks the
 * response contains a valid smoothed path with more points than input.
 */

#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <smooth_nav_msgs/srv/smooth_path.hpp>
#include <smooth_nav_msgs/msg/waypoint.hpp>

using namespace std::chrono_literals;

class SmootherServiceTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_smoother_client");
    client_ = node_->create_client<smooth_nav_msgs::srv::SmoothPath>(
        "/path_smoother/smooth_path");
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<smooth_nav_msgs::srv::SmoothPath>::SharedPtr client_;
};

TEST_F(SmootherServiceTest, ServiceAvailable) {
  ASSERT_TRUE(client_->wait_for_service(10s))
      << "SmoothPath service not available after 10 s";
}

TEST_F(SmootherServiceTest, SmoothsWaypoints) {
  ASSERT_TRUE(client_->wait_for_service(10s));

  auto request = std::make_shared<smooth_nav_msgs::srv::SmoothPath::Request>();

  // Build a simple 4-point L-shape
  std::vector<std::pair<double, double>> coords = {
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}};
  for (auto &[x, y] : coords) {
    smooth_nav_msgs::msg::Waypoint wp;
    wp.x = x;
    wp.y = y;
    request->waypoints.waypoints.push_back(wp);
  }

  auto future = client_->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future, 10s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_GT(response->smoothed_path.points.size(), coords.size())
      << "Smoothed path should have more points than raw waypoints";
}

TEST_F(SmootherServiceTest, RejectsLessThanTwoPoints) {
  ASSERT_TRUE(client_->wait_for_service(10s));

  auto request = std::make_shared<smooth_nav_msgs::srv::SmoothPath::Request>();
  smooth_nav_msgs::msg::Waypoint wp;
  wp.x = 0.0;
  wp.y = 0.0;
  request->waypoints.waypoints.push_back(wp);

  auto future = client_->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future, 10s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_FALSE(response->success)
      << "Service should reject fewer than 2 waypoints";
}
