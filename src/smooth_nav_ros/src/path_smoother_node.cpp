/**
 * @file path_smoother_node.cpp
 * @brief ROS 2 service node wrapping smooth_nav_core path smoothers.
 *
 * Provides:
 *   - ~/smooth_path service (smooth_nav_msgs/srv/SmoothPath)
 *   - /smoothed_path topic (nav_msgs/Path) for RViz visualization
 *   - /original_waypoints topic (visualization_msgs/MarkerArray)
 *
 * Parameters:
 *   - smoother_type: "cubic_spline" | "bspline" (default: cubic_spline)
 *   - num_smooth_points: int (default: 200)
 *   - bspline_weight_data, bspline_weight_smooth, bspline_tolerance, bspline_max_iterations
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

namespace smooth_nav_ros
{

class PathSmootherNode : public rclcpp::Node
{
public:
  PathSmootherNode()
  : Node("path_smoother_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("smoother_type", "cubic_spline");
    this->declare_parameter<int>("num_smooth_points", 200);
    this->declare_parameter<double>("bspline_weight_data", 0.1);
    this->declare_parameter<double>("bspline_weight_smooth", 0.3);
    this->declare_parameter<double>("bspline_tolerance", 1e-6);
    this->declare_parameter<int>("bspline_max_iterations", 10000);

    // Create smoother via factory
    std::string type = this->get_parameter("smoother_type").as_string();
    smoother_ = smooth_nav_core::SmootherFactory::create(type);

    if (!smoother_) {
      RCLCPP_ERROR(this->get_logger(), "Unknown smoother type: %s", type.c_str());
      throw std::runtime_error("Unknown smoother type: " + type);
    }

    // Configure BSpline params if applicable
    if (type == "bspline" || type == "gradient_descent") {
      auto * bspline = dynamic_cast<smooth_nav_core::BSplineSmoother *>(smoother_.get());
      if (bspline) {
        bspline->setParams(
          this->get_parameter("bspline_weight_data").as_double(),
          this->get_parameter("bspline_weight_smooth").as_double(),
          this->get_parameter("bspline_tolerance").as_double(),
          this->get_parameter("bspline_max_iterations").as_int()
        );
      }
    }

    RCLCPP_INFO(this->get_logger(), "Using smoother: %s", smoother_->name().c_str());

    // Create service
    service_ = this->create_service<smooth_nav_msgs::srv::SmoothPath>(
      "~/smooth_path",
      std::bind(&PathSmootherNode::handleSmoothPath, this,
                std::placeholders::_1, std::placeholders::_2));

    // Create visualization publishers
    smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/smoothed_path", 10);
    waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/original_waypoints", 10);

    RCLCPP_INFO(this->get_logger(), "PathSmootherNode ready. Service: ~/smooth_path");
  }

private:
  std::unique_ptr<smooth_nav_core::IPathSmoother> smoother_;
  rclcpp::Service<smooth_nav_msgs::srv::SmoothPath>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;

  void handleSmoothPath(
    const std::shared_ptr<smooth_nav_msgs::srv::SmoothPath::Request> request,
    std::shared_ptr<smooth_nav_msgs::srv::SmoothPath::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received smooth_path request with %zu waypoints",
                request->waypoints.waypoints.size());

    // Convert ROS message to core types
    std::vector<std::pair<double, double>> waypoints;
    for (const auto & wp : request->waypoints.waypoints) {
      waypoints.emplace_back(wp.x, wp.y);
    }

    int num_points = this->get_parameter("num_smooth_points").as_int();

    try {
      auto path = smoother_->smooth(waypoints, num_points);

      // Fill response
      response->smoothed_path.header.stamp = this->now();
      response->smoothed_path.header.frame_id = "odom";

      for (const auto & pt : path) {
        geometry_msgs::msg::Point point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = 0.0;
        response->smoothed_path.points.push_back(point);
        response->smoothed_path.curvatures.push_back(pt.curvature);
      }

      response->success = true;
      response->message = "Smoothed " + std::to_string(waypoints.size()) +
                          " waypoints → " + std::to_string(path.size()) +
                          " points using " + smoother_->name();

      // Publish visualization
      publishSmoothedPath(path);
      publishWaypointMarkers(waypoints);

      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("Smoothing failed: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

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
      ps.pose.position.z = 0.0;
      msg.poses.push_back(ps);
    }

    smooth_path_pub_->publish(msg);
  }

  void publishWaypointMarkers(
    const std::vector<std::pair<double, double>> & waypoints)
  {
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "odom";
      m.header.stamp = this->now();
      m.ns = "waypoints";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = waypoints[i].first;
      m.pose.position.y = waypoints[i].second;
      m.pose.position.z = 0.05;
      m.scale.x = m.scale.y = m.scale.z = 0.1;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      markers.markers.push_back(m);
    }
    waypoint_pub_->publish(markers);
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
