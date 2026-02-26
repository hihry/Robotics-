/**
 * @file path_smoother_node.cpp
 * @brief ROS 2 service node wrapping smooth_nav_core path smoothers.
 *
 * Architecture (real-world):
 *   ┌──────────────────┐   SmoothPath.srv    ┌──────────────────────┐
 *   │ waypoint_client   │ ───────────────────→│ path_smoother_node   │
 *   │ (or nav2)         │ ←─────────────────── │  • cubic_spline      │
 *   └──────────────────┘   SmoothedPath       │  • bspline           │
 *                                              └──────┬───────────────┘
 *                                                     │ publishes
 *                                              ┌──────▼───────────────┐
 *                                              │ /smoothed_path (Path)│
 *                                              │ /original_waypoints  │
 *                                              │ /curvature_markers   │
 *                                              └──────────────────────┘
 *
 * Provides:
 *   - ~/smooth_path service (smooth_nav_msgs/srv/SmoothPath)
 *   - /smoothed_path topic (nav_msgs/Path) for RViz
 *   - /original_waypoints topic (visualization_msgs/MarkerArray)
 *   - /curvature_markers topic (visualization_msgs/MarkerArray)
 *
 * Parameters (dynamically reconfigurable):
 *   - smoother_type: "cubic_spline" | "bspline"  (default: cubic_spline)
 *   - num_smooth_points: int                      (default: 200)
 *   - bspline_weight_data / bspline_weight_smooth / bspline_tolerance / bspline_max_iterations
 *
 * Real-world considerations:
 *   - Dynamic parameter reconfiguration — no restart needed to re-tune
 *   - Curvature visualisation — colour-mapped markers for debugging turns
 *   - Input validation with meaningful error messages
 *   - Thread-safe service callback (mutex around smoother)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "smooth_nav_msgs/srv/smooth_path.hpp"
#include "smooth_nav_msgs/msg/waypoint_array.hpp"
#include "smooth_nav_msgs/msg/smoothed_path.hpp"

#include "smooth_nav_core/path_smoother/smoother_factory.hpp"
#include "smooth_nav_core/path_smoother/bspline_smoother.hpp"

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>

namespace smooth_nav_ros
{

class PathSmootherNode : public rclcpp::Node
{
public:
  PathSmootherNode()
  : Node("path_smoother_node")
  {
    // ── Declare parameters ───────────────────────────────────────
    this->declare_parameter<std::string>("smoother_type", "cubic_spline");
    this->declare_parameter<int>("num_smooth_points", 200);
    this->declare_parameter<double>("bspline_weight_data", 0.1);
    this->declare_parameter<double>("bspline_weight_smooth", 0.3);
    this->declare_parameter<double>("bspline_tolerance", 1e-6);
    this->declare_parameter<int>("bspline_max_iterations", 10000);

    // Build initial smoother
    recreateSmoother();

    // ── Dynamic reconfigure callback ─────────────────────────────
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&PathSmootherNode::onParameterChange, this, std::placeholders::_1));

    // ── Service ──────────────────────────────────────────────────
    service_ = this->create_service<smooth_nav_msgs::srv::SmoothPath>(
      "~/smooth_path",
      std::bind(&PathSmootherNode::handleSmoothPath, this,
                std::placeholders::_1, std::placeholders::_2));

    // ── Visualisation publishers (latched for late-joining RViz) ─
    smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/smoothed_path", rclcpp::QoS(1).transient_local());
    waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/original_waypoints", rclcpp::QoS(1).transient_local());
    curvature_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/curvature_markers", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(this->get_logger(),
      "PathSmootherNode ready. Smoother: %s  Service: ~/smooth_path",
      smoother_->name().c_str());
  }

private:
  std::unique_ptr<smooth_nav_core::IPathSmoother> smoother_;
  std::mutex smoother_mutex_;

  rclcpp::Service<smooth_nav_msgs::srv::SmoothPath>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ═══════════════════════════════════════════════════════════════
  //  Dynamic reconfiguration
  // ═══════════════════════════════════════════════════════════════

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool needs_recreate = false;
    for (const auto & p : params) {
      if (p.get_name() == "smoother_type") {
        needs_recreate = true;
      } else if (p.get_name() == "num_smooth_points" && p.as_int() < 10) {
        result.successful = false;
        result.reason = "num_smooth_points must be >= 10";
        return result;
      }
    }

    if (needs_recreate) {
      try {
        recreateSmoother();
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = e.what();
      }
    }
    return result;
  }

  void recreateSmoother()
  {
    std::lock_guard<std::mutex> lock(smoother_mutex_);
    std::string type = this->get_parameter("smoother_type").as_string();
    smoother_ = smooth_nav_core::SmootherFactory::create(type);

    if (type == "bspline" || type == "gradient_descent") {
      auto * bsp = dynamic_cast<smooth_nav_core::BSplineSmoother *>(smoother_.get());
      if (bsp) {
        bsp->setParams(
          this->get_parameter("bspline_weight_data").as_double(),
          this->get_parameter("bspline_weight_smooth").as_double(),
          this->get_parameter("bspline_tolerance").as_double(),
          this->get_parameter("bspline_max_iterations").as_int());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Smoother (re)created: %s", smoother_->name().c_str());
  }

  // ═══════════════════════════════════════════════════════════════
  //  Service callback
  // ═══════════════════════════════════════════════════════════════

  void handleSmoothPath(
    const std::shared_ptr<smooth_nav_msgs::srv::SmoothPath::Request> request,
    std::shared_ptr<smooth_nav_msgs::srv::SmoothPath::Response> response)
  {
    auto & wps = request->waypoints.waypoints;
    RCLCPP_INFO(this->get_logger(), "SmoothPath request: %zu waypoints", wps.size());

    if (wps.size() < 2) {
      response->success = false;
      response->message = "Need >= 2 waypoints, got " + std::to_string(wps.size());
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // ROS → core
    std::vector<std::pair<double, double>> waypoints;
    waypoints.reserve(wps.size());
    for (const auto & wp : wps) {
      waypoints.emplace_back(wp.x, wp.y);
    }

    int num_pts = this->get_parameter("num_smooth_points").as_int();

    try {
      std::vector<smooth_nav_core::PathPoint> path;
      {
        std::lock_guard<std::mutex> lock(smoother_mutex_);
        path = smoother_->smooth(waypoints, num_pts);
      }

      // core → ROS
      response->smoothed_path.header.stamp = this->now();
      response->smoothed_path.header.frame_id = "odom";

      for (const auto & pt : path) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;  p.y = pt.y;  p.z = 0.0;
        response->smoothed_path.points.push_back(p);
        response->smoothed_path.curvatures.push_back(pt.curvature);
      }

      response->success = true;
      response->message = "Smoothed " + std::to_string(waypoints.size()) +
                          " waypoints → " + std::to_string(path.size()) +
                          " pts via " + smoother_->name();

      publishSmoothedPath(path);
      publishWaypointMarkers(waypoints);
      publishCurvatureMarkers(path);

      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("Smoothing failed: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  Visualisation helpers
  // ═══════════════════════════════════════════════════════════════

  void publishSmoothedPath(const std::vector<smooth_nav_core::PathPoint> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";

    for (const auto & pt : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = pt.x;
      ps.pose.position.y = pt.y;
      ps.pose.orientation.z = std::sin(pt.theta / 2.0);
      ps.pose.orientation.w = std::cos(pt.theta / 2.0);
      msg.poses.push_back(ps);
    }
    smooth_path_pub_->publish(msg);
  }

  void publishWaypointMarkers(
    const std::vector<std::pair<double, double>> & wps)
  {
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < wps.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "odom";
      m.header.stamp = this->now();
      m.ns = "waypoints";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = wps[i].first;
      m.pose.position.y = wps[i].second;
      m.pose.position.z = 0.05;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.1;
      m.color.r = 1.0f; m.color.a = 1.0f;
      markers.markers.push_back(m);

      // Text label
      visualization_msgs::msg::Marker t;
      t.header = m.header;
      t.ns = "waypoint_labels";
      t.id = static_cast<int>(i);
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = wps[i].first;
      t.pose.position.y = wps[i].second;
      t.pose.position.z = 0.20;
      t.pose.orientation.w = 1.0;
      t.scale.z = 0.08;
      t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 0.9f;
      t.text = "WP" + std::to_string(i);
      markers.markers.push_back(t);
    }
    waypoint_pub_->publish(markers);
  }

  /**
   * @brief Publish curvature as colour-mapped spheres along the path.
   *
   * Low curvature (straight) → green, High curvature (turn) → red.
   * Essential for tuning smoother parameters on a physical robot.
   */
  void publishCurvatureMarkers(const std::vector<smooth_nav_core::PathPoint> & path)
  {
    visualization_msgs::msg::MarkerArray markers;

    double max_k = 0.0;
    for (const auto & pt : path) max_k = std::max(max_k, std::abs(pt.curvature));
    if (max_k < 1e-9) max_k = 1.0;

    int step = std::max(1, static_cast<int>(path.size()) / 100);
    for (size_t i = 0; i < path.size(); i += step) {
      double ratio = std::abs(path[i].curvature) / max_k;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "odom";
      m.header.stamp = this->now();
      m.ns = "curvature";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = path[i].x;
      m.pose.position.y = path[i].y;
      m.pose.position.z = 0.03;
      m.pose.orientation.w = 1.0;
      double sz = 0.02 + ratio * 0.06;
      m.scale.x = m.scale.y = m.scale.z = sz;
      m.color.r = static_cast<float>(std::min(1.0, ratio * 2.0));
      m.color.g = static_cast<float>(std::min(1.0, (1.0 - ratio) * 2.0));
      m.color.a = 0.8f;
      markers.markers.push_back(m);
    }
    curvature_pub_->publish(markers);
  }
};

}  // namespace smooth_nav_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smooth_nav_ros::PathSmootherNode>());
  rclcpp::shutdown();
  return 0;
}
