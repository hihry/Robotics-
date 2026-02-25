/**
 * @file trajectory_tracker_node.cpp
 * @brief ROS 2 action server for trajectory tracking using Pure Pursuit + PID.
 *
 * Action: smooth_nav_msgs/action/ExecuteTrajectory
 *
 * Subscriptions:
 *   - /odom (nav_msgs/Odometry): Robot odometry
 *
 * Publishers:
 *   - /cmd_vel (geometry_msgs/Twist): Velocity commands
 *   - /tracking_error (std_msgs/Float64): Cross-track error
 *   - /actual_path (nav_msgs/Path): Actual path taken
 *   - /controller_diagnostics (smooth_nav_msgs/ControllerDiagnostics)
 *
 * Parameters:
 *   - look_ahead_distance, max_linear_velocity, max_angular_velocity,
 *     goal_tolerance, pid_kp, pid_ki, pid_kd, control_rate
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>

#include "smooth_nav_msgs/action/execute_trajectory.hpp"
#include "smooth_nav_msgs/msg/controller_diagnostics.hpp"
#include "smooth_nav_msgs/msg/trajectory.hpp"

#include "smooth_nav_core/controller/pure_pursuit_controller.hpp"
#include "smooth_nav_core/math/types.hpp"

#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;
using ExecuteTrajectory = smooth_nav_msgs::action::ExecuteTrajectory;
using GoalHandleET = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

namespace smooth_nav_controller
{

class TrajectoryTrackerNode : public rclcpp::Node
{
public:
  TrajectoryTrackerNode()
  : Node("trajectory_tracker_node")
  {
    // Parameters
    this->declare_parameter<double>("look_ahead_distance", 0.3);
    this->declare_parameter<double>("max_linear_velocity", 0.18);
    this->declare_parameter<double>("max_angular_velocity", 2.0);
    this->declare_parameter<double>("goal_tolerance", 0.08);
    this->declare_parameter<double>("pid_kp", 1.0);
    this->declare_parameter<double>("pid_ki", 0.0);
    this->declare_parameter<double>("pid_kd", 0.1);
    this->declare_parameter<double>("control_rate", 20.0);

    // Configure controller
    controller_.setLookAheadDistance(
      this->get_parameter("look_ahead_distance").as_double());
    controller_.setMaxLinearVelocity(
      this->get_parameter("max_linear_velocity").as_double());
    controller_.setMaxAngularVelocity(
      this->get_parameter("max_angular_velocity").as_double());
    controller_.setGoalTolerance(
      this->get_parameter("goal_tolerance").as_double());
    controller_.setPIDGains(
      this->get_parameter("pid_kp").as_double(),
      this->get_parameter("pid_ki").as_double(),
      this->get_parameter("pid_kd").as_double());

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/tracking_error", 10);
    actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
    diag_pub_ = this->create_publisher<smooth_nav_msgs::msg::ControllerDiagnostics>(
      "/controller_diagnostics", 10);

    // Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryTrackerNode::odomCallback, this, std::placeholders::_1));

    // Action server
    action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this,
      "~/execute_trajectory",
      std::bind(&TrajectoryTrackerNode::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTrackerNode::handleCancel, this,
                std::placeholders::_1),
      std::bind(&TrajectoryTrackerNode::handleAccepted, this,
                std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "TrajectoryTrackerNode ready. Action: ~/execute_trajectory");
  }

private:
  smooth_nav_core::PurePursuitController controller_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<smooth_nav_msgs::msg::ControllerDiagnostics>::SharedPtr diag_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;

  // State
  smooth_nav_core::Pose2D current_pose_;
  bool odom_received_ = false;
  std::mutex odom_mutex_;

  // ──── Odometry ────────────────────────────────────────────────────────

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose_.theta = yaw;

    odom_received_ = true;
  }

  // ──── Action Server Callbacks ─────────────────────────────────────────

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ExecuteTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received trajectory execution goal with %zu points",
      goal->trajectory.points.size());

    if (goal->trajectory.points.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Rejecting: trajectory has < 2 points");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleET> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Trajectory execution cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleET> goal_handle)
  {
    // Execute in a separate thread
    std::thread{std::bind(&TrajectoryTrackerNode::executeTrajectory, this,
                           goal_handle)}.detach();
  }

  // ──── Trajectory Execution ────────────────────────────────────────────

  void executeTrajectory(const std::shared_ptr<GoalHandleET> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution...");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();

    // Convert ROS message to core TrajectoryPoint
    std::vector<smooth_nav_core::TrajectoryPoint> trajectory;
    for (const auto & pt_msg : goal->trajectory.points) {
      smooth_nav_core::TrajectoryPoint tp;
      tp.x = pt_msg.x;
      tp.y = pt_msg.y;
      tp.theta = pt_msg.heading;
      tp.v = pt_msg.velocity;
      tp.omega = pt_msg.angular_velocity;
      tp.curvature = pt_msg.curvature;
      tp.arc_length = pt_msg.arc_length;
      tp.timestamp = pt_msg.stamp.sec + pt_msg.stamp.nanosec * 1e-9;
      trajectory.push_back(tp);
    }

    controller_.reset();
    controller_.setTrajectory(trajectory);

    nav_msgs::msg::Path actual_path_msg;
    actual_path_msg.header.frame_id = "odom";

    double control_rate = this->get_parameter("control_rate").as_double();
    rclcpp::Rate rate(control_rate);
    auto start_time = this->now();

    double max_cross_track = 0.0;
    double sum_cross_track = 0.0;
    size_t control_steps = 0;

    // Wait for odometry
    while (!odom_received_ && rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for /odom...");
      rate.sleep();
    }

    // ──── Control Loop ────
    while (rclcpp::ok() && !controller_.isComplete()) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        stopRobot();
        result->success = false;
        result->total_time = (this->now() - start_time).seconds();
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");
        return;
      }

      // Get pose (thread-safe)
      smooth_nav_core::Pose2D pose;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pose = current_pose_;
      }

      // Compute control
      double current_time = this->now().seconds();
      auto cmd = controller_.computeControl(pose, current_time);

      // Publish velocity command
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = cmd.linear;
      twist.angular.z = cmd.angular;
      cmd_vel_pub_->publish(twist);

      // Diagnostics
      double cte = controller_.getCrossTrackError();
      max_cross_track = std::max(max_cross_track, cte);
      sum_cross_track += cte;
      control_steps++;

      // Publish error
      auto error_msg = std_msgs::msg::Float64();
      error_msg.data = cte;
      error_pub_->publish(error_msg);

      // Publish diagnostics
      auto diag = smooth_nav_msgs::msg::ControllerDiagnostics();
      diag.header.stamp = this->now();
      diag.cross_track_error = cte;
      diag.heading_error = controller_.getHeadingError();
      diag.cmd_linear_velocity = cmd.linear;
      diag.cmd_angular_velocity = cmd.angular;
      double total_pts = static_cast<double>(trajectory.size());
      double nearest_idx = static_cast<double>(controller_.getNearestIndex());
      diag.progress_percent = (total_pts > 0) ? (nearest_idx / total_pts) * 100.0 : 0.0;
      diag.nearest_trajectory_index = static_cast<int32_t>(controller_.getNearestIndex());
      diag_pub_->publish(diag);

      // Publish feedback
      feedback->progress_percent = static_cast<float>(diag.progress_percent);
      feedback->cross_track_error = cte;
      feedback->heading_error = controller_.getHeadingError();
      goal_handle->publish_feedback(feedback);

      // Record actual path
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = this->now();
      ps.header.frame_id = "odom";
      ps.pose.position.x = pose.x;
      ps.pose.position.y = pose.y;
      actual_path_msg.poses.push_back(ps);
      actual_path_msg.header.stamp = this->now();
      actual_path_pub_->publish(actual_path_msg);

      rate.sleep();
    }

    // ──── Done ────
    stopRobot();

    result->success = true;
    result->total_time = (this->now() - start_time).seconds();
    result->max_cross_track_error = max_cross_track;
    result->mean_cross_track_error = (control_steps > 0)
      ? sum_cross_track / control_steps : 0.0;

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(),
      "Trajectory complete! Duration: %.1f s, Max CTE: %.4f m, Mean CTE: %.4f m",
      result->total_time, result->max_cross_track_error, result->mean_cross_track_error);
  }

  void stopRobot()
  {
    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist);
  }
};

}  // namespace smooth_nav_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smooth_nav_controller::TrajectoryTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
