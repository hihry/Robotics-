/**
 * @file trajectory_generator_node.cpp
 * @brief ROS 2 service node — converts smoothed paths into time-parameterised trajectories.
 *
 * Provides:
 *   - ~/generate_trajectory service (smooth_nav_msgs/srv/GenerateTrajectory)
 *   - /trajectory_path topic (nav_msgs/Path) for RViz trajectory visualisation
 *   - /velocity_profile topic (visualization_msgs/MarkerArray) — velocity colour map
 *
 * Parameters (dynamically reconfigurable):
 *   - generator_type: "trapezoidal" | "constant"       (default: trapezoidal)
 *   - max_velocity:   double                            (default: 0.18)
 *   - max_acceleration: double                          (default: 0.5)
 *   - max_lateral_acceleration: double                  (default: 0.5)
 *   - time_step: double                                 (default: 0.05)
 *
 * Real-world considerations:
 *   - Curvature-based speed limiting (max_lateral_acceleration) exposed as ROS param
 *   - Dynamic reconfiguration — tune on the fly in rqt_reconfigure
 *   - Input validation & heading computation from path geometry
 *   - Trajectory visualisation for debugging velocity profiles
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "smooth_nav_msgs/srv/generate_trajectory.hpp"
#include "smooth_nav_msgs/msg/smoothed_path.hpp"
#include "smooth_nav_msgs/msg/trajectory.hpp"

#include "smooth_nav_core/trajectory_generator/trapezoidal_velocity_generator.hpp"
#include "smooth_nav_core/trajectory_generator/constant_velocity_generator.hpp"
#include "smooth_nav_core/trajectory_generator/i_trajectory_generator.hpp"
#include "smooth_nav_core/math/types.hpp"

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>

namespace smooth_nav_ros
{

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode()
  : Node("trajectory_generator_node")
  {
    // ── Declare parameters ───────────────────────────────────────
    this->declare_parameter<std::string>("generator_type", "trapezoidal");
    this->declare_parameter<double>("max_velocity", 0.18);
    this->declare_parameter<double>("max_acceleration", 0.5);
    this->declare_parameter<double>("max_lateral_acceleration", 0.5);
    this->declare_parameter<double>("time_step", 0.05);

    createGenerator();

    // ── Dynamic reconfigure callback ─────────────────────────────
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&TrajectoryGeneratorNode::onParameterChange, this,
                std::placeholders::_1));

    // ── Service ──────────────────────────────────────────────────
    service_ = this->create_service<smooth_nav_msgs::srv::GenerateTrajectory>(
      "~/generate_trajectory",
      std::bind(&TrajectoryGeneratorNode::handleGenerate, this,
                std::placeholders::_1, std::placeholders::_2));

    // ── Visualisation publishers ─────────────────────────────────
    traj_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/trajectory_path", rclcpp::QoS(1).transient_local());
    vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/velocity_profile", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(this->get_logger(),
      "TrajectoryGeneratorNode ready. Type: %s  v_max=%.2f  a_max=%.2f  a_lat=%.2f",
      this->get_parameter("generator_type").as_string().c_str(),
      this->get_parameter("max_velocity").as_double(),
      this->get_parameter("max_acceleration").as_double(),
      this->get_parameter("max_lateral_acceleration").as_double());
  }

private:
  std::unique_ptr<smooth_nav_core::ITrajectoryGenerator> generator_;
  std::mutex gen_mutex_;

  rclcpp::Service<smooth_nav_msgs::srv::GenerateTrajectory>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vel_marker_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ═══════════════════════════════════════════════════════════════
  //  Dynamic reconfiguration
  // ═══════════════════════════════════════════════════════════════

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      if (p.get_name() == "max_velocity" && p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "max_velocity must be > 0";
        return result;
      }
      if (p.get_name() == "max_acceleration" && p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "max_acceleration must be > 0";
        return result;
      }
      if (p.get_name() == "time_step" && p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "time_step must be > 0";
        return result;
      }
    }

    // Rebuild generator with new params
    try {
      createGenerator();
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  }

  void createGenerator()
  {
    std::lock_guard<std::mutex> lock(gen_mutex_);

    std::string type = this->get_parameter("generator_type").as_string();
    double v_max = this->get_parameter("max_velocity").as_double();
    double a_max = this->get_parameter("max_acceleration").as_double();
    double a_lat = this->get_parameter("max_lateral_acceleration").as_double();
    double dt    = this->get_parameter("time_step").as_double();

    if (type == "constant") {
      auto gen = std::make_unique<smooth_nav_core::ConstantVelocityGenerator>();
      gen->setMaxVelocity(v_max);
      gen->setTimeStep(dt);
      if (a_lat > 0.0) gen->setMaxLateralAcceleration(a_lat);
      generator_ = std::move(gen);
    } else {
      auto gen = std::make_unique<smooth_nav_core::TrapezoidalVelocityGenerator>();
      gen->setMaxVelocity(v_max);
      gen->setMaxAcceleration(a_max);
      gen->setTimeStep(dt);
      if (a_lat > 0.0) gen->setMaxLateralAcceleration(a_lat);
      generator_ = std::move(gen);
    }

    RCLCPP_INFO(this->get_logger(), "Generator (re)created: %s", type.c_str());
  }

  // ═══════════════════════════════════════════════════════════════
  //  Service callback
  // ═══════════════════════════════════════════════════════════════

  void handleGenerate(
    const std::shared_ptr<smooth_nav_msgs::srv::GenerateTrajectory::Request> request,
    std::shared_ptr<smooth_nav_msgs::srv::GenerateTrajectory::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "GenerateTrajectory request: %zu path points", request->smoothed_path.points.size());

    if (request->smoothed_path.points.size() < 2) {
      response->success = false;
      response->message = "Need >= 2 path points, got " +
                          std::to_string(request->smoothed_path.points.size());
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // ROS SmoothedPath → core PathPoint[]
    std::vector<smooth_nav_core::PathPoint> path;
    auto & pts = request->smoothed_path.points;
    auto & curvs = request->smoothed_path.curvatures;
    path.reserve(pts.size());

    for (size_t i = 0; i < pts.size(); ++i) {
      smooth_nav_core::PathPoint pp;
      pp.x = pts[i].x;
      pp.y = pts[i].y;
      if (i < curvs.size()) pp.curvature = curvs[i];
      path.push_back(pp);
    }

    // Compute headings from path geometry (tangent direction)
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      path[i].theta = std::atan2(path[i + 1].y - path[i].y,
                                  path[i + 1].x - path[i].x);
    }
    if (path.size() >= 2) {
      path.back().theta = path[path.size() - 2].theta;
    }

    // Override generator limits from request if provided
    if (request->max_velocity > 0.0 || request->max_acceleration > 0.0) {
      try {
        std::lock_guard<std::mutex> lock(gen_mutex_);
        auto type = this->get_parameter("generator_type").as_string();
        if (type == "constant") {
          auto * g = dynamic_cast<smooth_nav_core::ConstantVelocityGenerator *>(generator_.get());
          if (g && request->max_velocity > 0.0) g->setMaxVelocity(request->max_velocity);
        } else {
          auto * g = dynamic_cast<smooth_nav_core::TrapezoidalVelocityGenerator *>(generator_.get());
          if (g) {
            if (request->max_velocity > 0.0) g->setMaxVelocity(request->max_velocity);
            if (request->max_acceleration > 0.0) g->setMaxAcceleration(request->max_acceleration);
          }
        }
      } catch (...) {}
    }

    try {
      std::vector<smooth_nav_core::TrajectoryPoint> traj;
      {
        std::lock_guard<std::mutex> lock(gen_mutex_);
        traj = generator_->generate(path);
      }

      // core → ROS
      response->trajectory.header.stamp = this->now();
      response->trajectory.header.frame_id = "odom";

      for (const auto & tp : traj) {
        smooth_nav_msgs::msg::TrajectoryPoint pt_msg;
        pt_msg.x = tp.x;
        pt_msg.y = tp.y;
        pt_msg.heading = tp.theta;
        pt_msg.velocity = tp.v;
        pt_msg.angular_velocity = tp.omega;
        pt_msg.curvature = tp.curvature;
        pt_msg.arc_length = tp.arc_length;
        pt_msg.stamp.sec = static_cast<int32_t>(tp.timestamp);
        pt_msg.stamp.nanosec = static_cast<uint32_t>(
          (tp.timestamp - static_cast<int32_t>(tp.timestamp)) * 1e9);
        response->trajectory.points.push_back(pt_msg);
      }

      double duration = traj.empty() ? 0.0 : traj.back().timestamp;
      response->success = true;
      response->message = "Generated " + std::to_string(traj.size()) +
                          " trajectory points via " + generator_->name() +
                          ". Duration: " + std::to_string(duration) + " s";

      // Visualise
      publishTrajectoryPath(traj);
      publishVelocityMarkers(traj);

      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("Generation failed: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  Visualisation
  // ═══════════════════════════════════════════════════════════════

  void publishTrajectoryPath(
    const std::vector<smooth_nav_core::TrajectoryPoint> & traj)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";

    for (const auto & tp : traj) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = tp.x;
      ps.pose.position.y = tp.y;
      ps.pose.orientation.z = std::sin(tp.theta / 2.0);
      ps.pose.orientation.w = std::cos(tp.theta / 2.0);
      msg.poses.push_back(ps);
    }
    traj_path_pub_->publish(msg);
  }

  /**
   * @brief Publish velocity-coloured line strip for the trajectory.
   *
   * Blue (slow) → Green (medium) → Red (fast).
   * This is essential for verifying that the trapezoidal profile and
   * curvature limiting are working correctly on the real robot.
   */
  void publishVelocityMarkers(
    const std::vector<smooth_nav_core::TrajectoryPoint> & traj)
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "odom";
    line.header.stamp = this->now();
    line.ns = "velocity_profile";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.02;

    double v_max = 0.01;
    for (const auto & tp : traj) v_max = std::max(v_max, tp.v);

    for (const auto & tp : traj) {
      geometry_msgs::msg::Point p;
      p.x = tp.x;  p.y = tp.y;  p.z = 0.01;
      line.points.push_back(p);

      double r = tp.v / v_max;
      std_msgs::msg::ColorRGBA c;
      if (r < 0.5) {
        c.r = 0.0f;
        c.g = static_cast<float>(r * 2.0);
        c.b = static_cast<float>(1.0 - r * 2.0);
      } else {
        c.r = static_cast<float>((r - 0.5) * 2.0);
        c.g = static_cast<float>(1.0 - (r - 0.5) * 2.0);
        c.b = 0.0f;
      }
      c.a = 0.9f;
      line.colors.push_back(c);
    }
    markers.markers.push_back(line);

    // Velocity arrows at sampled points
    int step = std::max(1, static_cast<int>(traj.size()) / 20);
    for (size_t i = 0; i < traj.size(); i += step) {
      const auto & tp = traj[i];
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "odom";
      arrow.header.stamp = this->now();
      arrow.ns = "velocity_arrows";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start, end;
      start.x = tp.x;  start.y = tp.y;  start.z = 0.02;
      double len = tp.v * 0.5;
      end.x = tp.x + len * std::cos(tp.theta);
      end.y = tp.y + len * std::sin(tp.theta);
      end.z = 0.02;
      arrow.points.push_back(start);
      arrow.points.push_back(end);

      arrow.scale.x = 0.015;
      arrow.scale.y = 0.03;
      arrow.scale.z = 0.03;
      arrow.color.r = 0.2f; arrow.color.g = 0.8f; arrow.color.b = 1.0f; arrow.color.a = 0.7f;
      markers.markers.push_back(arrow);
    }

    vel_marker_pub_->publish(markers);
  }
};

}  // namespace smooth_nav_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smooth_nav_ros::TrajectoryGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
