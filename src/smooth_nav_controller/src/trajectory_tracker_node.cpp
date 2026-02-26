// Copyright 2026 smooth_nav Authors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file trajectory_tracker_node.cpp
 * @brief ROS 2 action server — tracks a trajectory using Pure Pursuit + PID.
 *
 * Provides:
 *   - ~/execute_trajectory action (smooth_nav_msgs/action/ExecuteTrajectory)
 *
 * Publishes:
 *   - /cmd_vel_raw     (geometry_msgs/Twist)        — raw commands (safety_watchdog → /cmd_vel)
 *   - /actual_path     (nav_msgs/Path)              — odometry-based robot trail
 *   - /tracking_error  (std_msgs/Float64)           — cross-track error stream
 *   - /controller_diagnostics (smooth_nav_msgs/ControllerDiagnostics)
 *   - /velocity_command_markers (visualization_msgs/MarkerArray)
 *
 * Subscribes:
 *   - /odom  (nav_msgs/Odometry) — robot odometry
 *
 * Parameters (dynamically reconfigurable):
 *   - look_ahead_distance, adaptive_look_ahead_gain, goal_deceleration_radius
 *   - max_linear_velocity, max_angular_velocity
 *   - goal_tolerance
 *   - pid_kp, pid_ki, pid_kd
 *   - control_rate
 *
 * Real-world considerations:
 *   - Publishes to /cmd_vel_raw so safety_watchdog_node can enforce
 *     velocity/acceleration limits before actuating motors.
 *   - Adaptive look-ahead for stable tracking at varying speeds.
 *   - Goal deceleration zone prevents overshoot.
 *   - Thread-safe: mutex guards controller + trajectory state.
 *   - Diagnostics for live tuning in rqt_plot / PlotJuggler.
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "smooth_nav_msgs/action/execute_trajectory.hpp"
#include "smooth_nav_msgs/msg/controller_diagnostics.hpp"
#include "smooth_nav_msgs/msg/trajectory.hpp"
#include "smooth_nav_msgs/msg/trajectory_point.hpp"

#include "smooth_nav_core/controller/pure_pursuit_controller.hpp"
#include "smooth_nav_core/math/types.hpp"

namespace smooth_nav_controller
{

using ExecuteTrajectory = smooth_nav_msgs::action::ExecuteTrajectory;
using GoalHandleET = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

class TrajectoryTrackerNode : public rclcpp::Node
{
public:
  explicit TrajectoryTrackerNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("trajectory_tracker_node", opts)
  {
    // ── Declare parameters ──────────────────────────────────────
    this->declare_parameter<double>("look_ahead_distance", 0.3);
    this->declare_parameter<double>("adaptive_look_ahead_gain", 0.5);
    this->declare_parameter<double>("goal_deceleration_radius", 0.3);
    this->declare_parameter<double>("max_linear_velocity", 0.18);
    this->declare_parameter<double>("max_angular_velocity", 2.0);
    this->declare_parameter<double>("goal_tolerance", 0.08);
    this->declare_parameter<double>("pid_kp", 1.0);
    this->declare_parameter<double>("pid_ki", 0.0);
    this->declare_parameter<double>("pid_kd", 0.1);
    this->declare_parameter<double>("control_rate", 20.0);

    configureController();

    // ── Dynamic reconfigure ─────────────────────────────────────
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&TrajectoryTrackerNode::onParameterChange, this,
                std::placeholders::_1));

    // ── Action server ───────────────────────────────────────────
    action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this,
      "~/execute_trajectory",
      std::bind(&TrajectoryTrackerNode::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTrackerNode::handleCancel, this,
                std::placeholders::_1),
      std::bind(&TrajectoryTrackerNode::handleAccepted, this,
                std::placeholders::_1));

    // ── Publishers ──────────────────────────────────────────────
    cmd_pub_    = this->create_publisher<geometry_msgs::msg::Twist>(
                    "/cmd_vel_raw", 10);
    path_pub_   = this->create_publisher<nav_msgs::msg::Path>(
                    "/actual_path", rclcpp::QoS(1).transient_local());
    error_pub_  = this->create_publisher<std_msgs::msg::Float64>(
                    "/tracking_error", 10);
    diag_pub_   = this->create_publisher<smooth_nav_msgs::msg::ControllerDiagnostics>(
                    "/controller_diagnostics", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                    "/velocity_command_markers", rclcpp::QoS(1).transient_local());

    // ── Odometry subscriber ─────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryTrackerNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "TrajectoryTrackerNode ready. L_d=%.2f  v_max=%.2f  goal_tol=%.3f  rate=%.0f Hz",
      this->get_parameter("look_ahead_distance").as_double(),
      this->get_parameter("max_linear_velocity").as_double(),
      this->get_parameter("goal_tolerance").as_double(),
      this->get_parameter("control_rate").as_double());
  }

private:
  // ── Controller / state ─────────────────────────────────────────
  smooth_nav_core::PurePursuitController controller_;
  std::mutex ctrl_mutex_;

  smooth_nav_core::Pose2D current_pose_;
  bool has_odom_ = false;

  // ── ROS interfaces ─────────────────────────────────────────────
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  rclcpp::Publisher<smooth_nav_msgs::msg::ControllerDiagnostics>::SharedPtr diag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ══════════════════════════════════════════════════════════════
  //  Parameter reconfiguration
  // ══════════════════════════════════════════════════════════════

  void configureController()
  {
    std::lock_guard<std::mutex> lock(ctrl_mutex_);
    controller_.setLookAheadDistance(
      this->get_parameter("look_ahead_distance").as_double());
    controller_.setAdaptiveLookAheadGain(
      this->get_parameter("adaptive_look_ahead_gain").as_double());
    controller_.setGoalDecelerationRadius(
      this->get_parameter("goal_deceleration_radius").as_double());
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
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "look_ahead_distance" && p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "look_ahead_distance must be > 0";
        return result;
      }
      if (p.get_name() == "control_rate" && p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "control_rate must be > 0";
        return result;
      }
    }
    configureController();
    RCLCPP_INFO(this->get_logger(), "Controller params reconfigured");
    return result;
  }

  // ══════════════════════════════════════════════════════════════
  //  Odometry
  // ══════════════════════════════════════════════════════════════

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(ctrl_mutex_);
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    // Quaternion → yaw
    auto & q = msg->pose.pose.orientation;
    current_pose_.theta = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_odom_ = true;
  }

  // ══════════════════════════════════════════════════════════════
  //  Action callbacks
  // ══════════════════════════════════════════════════════════════

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTrajectory::Goal> goal)
  {
    if (goal->trajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejected: empty trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepted trajectory with %zu points",
                goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleET>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel requested — stopping");
    stopRobot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleET> goal_handle)
  {
    // Execute in a new thread to avoid blocking the executor
    std::thread{std::bind(&TrajectoryTrackerNode::execute, this,
                          std::placeholders::_1), goal_handle}.detach();
  }

  // ══════════════════════════════════════════════════════════════
  //  Main control loop
  // ══════════════════════════════════════════════════════════════

  void execute(const std::shared_ptr<GoalHandleET> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Execution started");

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
    auto result   = std::make_shared<ExecuteTrajectory::Result>();

    // ── Convert ROS → core TrajectoryPoint ──────────────────────
    std::vector<smooth_nav_core::TrajectoryPoint> core_traj;
    core_traj.reserve(goal->trajectory.points.size());
    for (const auto & rp : goal->trajectory.points) {
      smooth_nav_core::TrajectoryPoint tp;
      tp.x         = rp.x;
      tp.y         = rp.y;
      tp.theta     = rp.heading;
      tp.v         = rp.velocity;
      tp.omega     = rp.angular_velocity;
      tp.curvature = rp.curvature;
      tp.arc_length = rp.arc_length;
      tp.timestamp = rp.stamp.sec + rp.stamp.nanosec * 1e-9;
      core_traj.push_back(tp);
    }

    {
      std::lock_guard<std::mutex> lock(ctrl_mutex_);
      controller_.setTrajectory(core_traj);
    }

    // ── Actual path for visualisation ──────────────────────────
    nav_msgs::msg::Path actual_path;
    actual_path.header.frame_id = "odom";

    // ── Wait for first odom ────────────────────────────────────
    {
      auto wait_start = this->now();
      while (!has_odom_ && rclcpp::ok()) {
        if ((this->now() - wait_start).seconds() > 5.0) {
          RCLCPP_ERROR(this->get_logger(), "No odometry after 5 s — aborting");
          result->success = false;
          result->total_time = 0.0;
          goal_handle->abort(result);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    // ── Control loop ───────────────────────────────────────────
    double rate_hz = this->get_parameter("control_rate").as_double();
    rclcpp::Rate rate(rate_hz);

    double start_time = this->now().seconds();
    double wall_time  = 0.0;
    std::vector<double> cte_log;
    cte_log.reserve(core_traj.size());

    while (rclcpp::ok()) {
      // Check cancel
      if (goal_handle->is_canceling()) {
        stopRobot();
        result->success = false;
        result->total_time = wall_time;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory cancelled");
        return;
      }

      wall_time = this->now().seconds() - start_time;

      smooth_nav_core::Velocity2D cmd;
      smooth_nav_core::ControllerState state;
      {
        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        cmd   = controller_.computeControl(current_pose_, wall_time);
        state = controller_.getState();
      }

      // Publish command
      geometry_msgs::msg::Twist twist;
      twist.linear.x  = cmd.linear;
      twist.angular.z = cmd.angular;
      cmd_pub_->publish(twist);

      // Track errors
      cte_log.push_back(std::abs(state.cross_track_error));

      // Publish tracking error
      std_msgs::msg::Float64 err_msg;
      err_msg.data = state.cross_track_error;
      error_pub_->publish(err_msg);

      // Publish diagnostics
      smooth_nav_msgs::msg::ControllerDiagnostics diag;
      diag.header.stamp = this->now();
      diag.cross_track_error = state.cross_track_error;
      diag.heading_error = state.heading_error;
      diag.cmd_linear_velocity = cmd.linear;
      diag.cmd_angular_velocity = cmd.angular;
      diag.progress_percent = state.progress * 100.0;
      diag.nearest_trajectory_index = static_cast<uint32_t>(state.nearest_index);
      diag_pub_->publish(diag);

      // Append actual path
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = this->now();
      ps.header.frame_id = "odom";
      {
        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        ps.pose.position.x = current_pose_.x;
        ps.pose.position.y = current_pose_.y;
        ps.pose.orientation.z = std::sin(current_pose_.theta / 2.0);
        ps.pose.orientation.w = std::cos(current_pose_.theta / 2.0);
      }
      actual_path.poses.push_back(ps);
      actual_path.header.stamp = this->now();
      path_pub_->publish(actual_path);

      // Publish velocity arrow
      publishVelocityArrow(current_pose_, cmd);

      // Feedback
      feedback->progress_percent    = state.progress * 100.0;
      feedback->cross_track_error   = state.cross_track_error;
      feedback->heading_error       = state.heading_error;
      goal_handle->publish_feedback(feedback);

      // Done?
      if (state.trajectory_complete) {
        RCLCPP_INFO(this->get_logger(), "Trajectory complete!");
        break;
      }

      rate.sleep();
    }

    // ── Stop & publish result ──────────────────────────────────
    stopRobot();

    result->success = true;
    result->total_time = wall_time;
    result->max_cross_track_error = cte_log.empty() ? 0.0 :
      *std::max_element(cte_log.begin(), cte_log.end());
    result->mean_cross_track_error = cte_log.empty() ? 0.0 :
      std::accumulate(cte_log.begin(), cte_log.end(), 0.0) / cte_log.size();

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(),
      "Finished. Time=%.2f s  Max CTE=%.4f  Mean CTE=%.4f",
      result->total_time, result->max_cross_track_error,
      result->mean_cross_track_error);
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  // ══════════════════════════════════════════════════════════════
  //  Visualisation helpers
  // ══════════════════════════════════════════════════════════════

  void publishVelocityArrow(
    const smooth_nav_core::Pose2D & pose,
    const smooth_nav_core::Velocity2D & cmd)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "odom";
    arrow.header.stamp = this->now();
    arrow.ns = "cmd_vel_arrow";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.lifetime = rclcpp::Duration::from_seconds(0.2);

    geometry_msgs::msg::Point start, end;
    start.x = pose.x; start.y = pose.y; start.z = 0.05;
    double len = std::abs(cmd.linear) * 0.5;
    end.x = pose.x + len * std::cos(pose.theta);
    end.y = pose.y + len * std::sin(pose.theta);
    end.z = 0.05;
    arrow.points.push_back(start);
    arrow.points.push_back(end);

    arrow.scale.x = 0.02;
    arrow.scale.y = 0.04;
    arrow.scale.z = 0.04;

    // Colour: green → forward, red → backwards
    arrow.color.r = cmd.linear < 0 ? 1.0f : 0.0f;
    arrow.color.g = cmd.linear >= 0 ? 1.0f : 0.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 0.9f;

    markers.markers.push_back(arrow);
    marker_pub_->publish(markers);
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
