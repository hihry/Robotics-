/**
 * @file trajectory_node.cpp
 * @brief Main ROS2 node that integrates path smoothing, trajectory generation,
 *        and trajectory tracking for TurtleBot3.
 *
 * This node:
 *   1. Loads waypoints from a YAML config (or parameter)
 *   2. Smooths the path using cubic spline interpolation
 *   3. Generates a time-parameterized trajectory with trapezoidal velocity profile
 *   4. Tracks the trajectory using Pure Pursuit + PID controller
 *   5. Publishes velocity commands to /cmd_vel
 *   6. Publishes visualization markers and tracking error
 *
 * Subscriptions:
 *   - /odom (nav_msgs/Odometry): Robot odometry
 *
 * Publishers:
 *   - /cmd_vel (geometry_msgs/Twist): Velocity commands
 *   - /smoothed_path (nav_msgs/Path): Smoothed path visualization
 *   - /original_waypoints (visualization_msgs/MarkerArray): Waypoint markers
 *   - /tracking_error (std_msgs/Float64): Cross-track error
 *   - /actual_path (nav_msgs/Path): Actual path taken by robot
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>

#include "trajectory_control/path_smoother.hpp"
#include "trajectory_control/trajectory_generator.hpp"
#include "trajectory_control/trajectory_controller.hpp"
#include "trajectory_control/types.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace trajectory_control
{

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode() : Node("trajectory_node")
  {
    // ---- Declare Parameters ----
    this->declare_parameter<std::vector<double>>("waypoints_x",
      {0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 3.0, 2.0, 1.0, 0.0});
    this->declare_parameter<std::vector<double>>("waypoints_y",
      {0.0, 0.0, 0.5, 0.5, 0.0, -1.0, -1.5, -1.5, -1.0, 0.0});

    // Smoother params
    this->declare_parameter<std::string>("smoothing_method", "cubic_spline");
    this->declare_parameter<int>("num_smooth_points", 200);
    this->declare_parameter<double>("smooth_weight_data", 0.1);
    this->declare_parameter<double>("smooth_weight_smooth", 0.3);

    // Trajectory generator params
    this->declare_parameter<double>("max_velocity", 0.18);
    this->declare_parameter<double>("max_acceleration", 0.5);
    this->declare_parameter<std::string>("velocity_profile", "trapezoidal");
    this->declare_parameter<double>("trajectory_dt", 0.05);

    // Controller params
    this->declare_parameter<double>("look_ahead_distance", 0.3);
    this->declare_parameter<double>("max_angular_velocity", 2.0);
    this->declare_parameter<double>("goal_tolerance", 0.08);
    this->declare_parameter<double>("pid_kp", 1.0);
    this->declare_parameter<double>("pid_ki", 0.0);
    this->declare_parameter<double>("pid_kd", 0.1);

    // ---- Create Publishers ----
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);
    actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
    waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/original_waypoints", 10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/tracking_error", 10);

    // ---- Create Subscriber ----
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryNode::odomCallback, this, std::placeholders::_1));

    // ---- Control Loop Timer (20 Hz) ----
    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&TrajectoryNode::controlLoop, this));

    // ---- Visualization Timer (2 Hz) ----
    viz_timer_ = this->create_wall_timer(
      500ms, std::bind(&TrajectoryNode::publishVisualization, this));

    // ---- Initialize Pipeline ----
    initializePipeline();

    RCLCPP_INFO(this->get_logger(),
      "TrajectoryNode initialized. Waiting for /odom data...");
  }

private:
  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // Algorithm components
  PathSmoother smoother_;
  TrajectoryGenerator generator_;
  TrajectoryController controller_;

  // State
  Pose2D current_pose_;
  bool odom_received_ = false;
  bool pipeline_ready_ = false;

  // Waypoints & paths
  std::vector<std::pair<double, double>> waypoints_;
  std::vector<Pose2D> smooth_path_;
  std::vector<TrajectoryPoint> trajectory_;
  nav_msgs::msg::Path actual_path_msg_;

  /**
   * @brief Initialize the full pipeline: smooth → generate → configure controller.
   */
  void initializePipeline()
  {
    // Load waypoints from parameters
    auto wx = this->get_parameter("waypoints_x").as_double_array();
    auto wy = this->get_parameter("waypoints_y").as_double_array();

    if (wx.size() != wy.size() || wx.size() < 2) {
      RCLCPP_ERROR(this->get_logger(),
        "Invalid waypoints: x has %zu points, y has %zu points. Need >= 2.",
        wx.size(), wy.size());
      return;
    }

    waypoints_.clear();
    for (size_t i = 0; i < wx.size(); ++i) {
      waypoints_.emplace_back(wx[i], wy[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());

    // Configure smoother
    std::string method = this->get_parameter("smoothing_method").as_string();
    if (method == "gradient_descent") {
      smoother_.setMethod(SmoothingMethod::GRADIENT_DESCENT);
    } else {
      smoother_.setMethod(SmoothingMethod::CUBIC_SPLINE);
    }
    smoother_.setNumPoints(this->get_parameter("num_smooth_points").as_int());
    smoother_.setGradientDescentParams(
      this->get_parameter("smooth_weight_data").as_double(),
      this->get_parameter("smooth_weight_smooth").as_double(),
      1e-6, 10000);

    // Smooth the path
    try {
      smooth_path_ = smoother_.smooth(waypoints_);
      RCLCPP_INFO(this->get_logger(),
        "Path smoothed: %zu input → %zu output points",
        waypoints_.size(), smooth_path_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Path smoothing failed: %s", e.what());
      return;
    }

    // Configure trajectory generator
    generator_.setMaxVelocity(this->get_parameter("max_velocity").as_double());
    generator_.setMaxAcceleration(this->get_parameter("max_acceleration").as_double());
    generator_.setTimeStep(this->get_parameter("trajectory_dt").as_double());

    std::string profile = this->get_parameter("velocity_profile").as_string();
    if (profile == "constant") {
      generator_.setProfileType(VelocityProfileType::CONSTANT);
    } else {
      generator_.setProfileType(VelocityProfileType::TRAPEZOIDAL);
    }

    // Generate trajectory
    try {
      trajectory_ = generator_.generate(smooth_path_);
      RCLCPP_INFO(this->get_logger(),
        "Trajectory generated: %zu points, duration: %.1f s",
        trajectory_.size(),
        trajectory_.empty() ? 0.0 : trajectory_.back().timestamp);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed: %s", e.what());
      return;
    }

    // Configure controller
    controller_.setLookAheadDistance(
      this->get_parameter("look_ahead_distance").as_double());
    controller_.setMaxLinearVelocity(
      this->get_parameter("max_velocity").as_double());
    controller_.setMaxAngularVelocity(
      this->get_parameter("max_angular_velocity").as_double());
    controller_.setGoalTolerance(
      this->get_parameter("goal_tolerance").as_double());
    controller_.setPIDGains(
      this->get_parameter("pid_kp").as_double(),
      this->get_parameter("pid_ki").as_double(),
      this->get_parameter("pid_kd").as_double());
    controller_.setTrajectory(trajectory_);

    pipeline_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Pipeline ready. Trajectory tracking will start.");
  }

  /**
   * @brief Odometry callback — update current robot pose.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    // Extract yaw from quaternion
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

  /**
   * @brief Main control loop — compute and publish velocity commands.
   */
  void controlLoop()
  {
    if (!pipeline_ready_ || !odom_received_) {
      return;
    }

    if (controller_.isTrajectoryComplete()) {
      // Send stop command
      auto twist = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(twist);

      static bool logged = false;
      if (!logged) {
        RCLCPP_INFO(this->get_logger(),
          "Trajectory complete! Final cross-track error: %.4f m",
          controller_.getCrossTrackError());
        logged = true;
      }
      return;
    }

    // Compute control
    double current_time = this->now().seconds();
    auto [v, omega] = controller_.computeControl(current_pose_, current_time);

    // Publish velocity command
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = v;
    twist.angular.z = omega;
    cmd_vel_pub_->publish(twist);

    // Publish tracking error
    auto error_msg = std_msgs::msg::Float64();
    error_msg.data = controller_.getCrossTrackError();
    error_pub_->publish(error_msg);

    // Record actual path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = current_pose_.x;
    pose_stamped.pose.position.y = current_pose_.y;
    actual_path_msg_.poses.push_back(pose_stamped);
  }

  /**
   * @brief Publish visualization markers (smoothed path + waypoints).
   */
  void publishVisualization()
  {
    if (!pipeline_ready_) return;

    auto now = this->now();

    // --- Smoothed path ---
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = "odom";

    for (const auto & pt : smooth_path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = pt.x;
      ps.pose.position.y = pt.y;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    smooth_path_pub_->publish(path_msg);

    // --- Actual path ---
    actual_path_msg_.header.stamp = now;
    actual_path_msg_.header.frame_id = "odom";
    actual_path_pub_->publish(actual_path_msg_);

    // --- Original waypoints as markers ---
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = "odom";
      marker.ns = "waypoints";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = waypoints_[i].first;
      marker.pose.position.y = waypoints_[i].second;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      // Red color for waypoints
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);
    }
    waypoint_pub_->publish(marker_array);
  }
};

}  // namespace trajectory_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trajectory_control::TrajectoryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
