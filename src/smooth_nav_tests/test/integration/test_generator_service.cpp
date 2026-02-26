/**
 * @file test_generator_service.cpp
 * @brief Integration test — verifies the /trajectory_generator/generate_trajectory
 *        service responds correctly given a smoothed path.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <smooth_nav_msgs/srv/generate_trajectory.hpp>
#include <smooth_nav_msgs/msg/smoothed_path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class GeneratorServiceTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_generator_client");
    client_ = node_->create_client<smooth_nav_msgs::srv::GenerateTrajectory>(
        "/trajectory_generator/generate_trajectory");
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<smooth_nav_msgs::srv::GenerateTrajectory>::SharedPtr client_;
};

TEST_F(GeneratorServiceTest, ServiceAvailable) {
  ASSERT_TRUE(client_->wait_for_service(10s))
      << "GenerateTrajectory service not available after 10 s";
}

TEST_F(GeneratorServiceTest, GeneratesTrajectory) {
  ASSERT_TRUE(client_->wait_for_service(10s));

  auto request =
      std::make_shared<smooth_nav_msgs::srv::GenerateTrajectory::Request>();

  // Build a straight-line smoothed path
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = i * 0.3;
    pt.y = 0.0;
    pt.z = 0.0;
    request->smoothed_path.points.push_back(pt);
  }
  request->max_velocity = 0.22;
  request->max_acceleration = 0.5;

  auto future = client_->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future, 10s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_GT(response->trajectory.points.size(), 0u)
      << "Trajectory should contain at least one point";

  // Verify timestamps are monotonically increasing
  for (size_t i = 1; i < response->trajectory.points.size(); ++i) {
    double t_i = response->trajectory.points[i].stamp.sec +
                 response->trajectory.points[i].stamp.nanosec * 1e-9;
    double t_prev = response->trajectory.points[i - 1].stamp.sec +
                    response->trajectory.points[i - 1].stamp.nanosec * 1e-9;
    EXPECT_GE(t_i, t_prev);
  }
}

TEST_F(GeneratorServiceTest, RejectsEmptyPath) {
  ASSERT_TRUE(client_->wait_for_service(10s));

  auto request =
      std::make_shared<smooth_nav_msgs::srv::GenerateTrajectory::Request>();
  request->max_velocity = 0.22;
  request->max_acceleration = 0.5;

  auto future = client_->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future, 10s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_FALSE(response->success)
      << "Service should reject an empty smoothed path";
}
