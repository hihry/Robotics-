/**
 * @file trajectory_generator_node.cpp
 * @brief ROS 2 service node wrapping smooth_nav_core trajectory generators.
 *
 * Provides:
 *   - ~/generate_trajectory service (smooth_nav_msgs/srv/GenerateTrajectory)
 *
 * Parameters:
 *   - generator_type: "trapezoidal" | "constant" (default: trapezoidal)
 *   - max_velocity, max_acceleration, time_step
 */

#include <rclcpp/rclcpp.hpp>

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

namespace smooth_nav_ros
{

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode()
  : Node("trajectory_generator_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("generator_type", "trapezoidal");
    this->declare_parameter<double>("max_velocity", 0.18);
    this->declare_parameter<double>("max_acceleration", 0.5);
    this->declare_parameter<double>("time_step", 0.05);

    createGenerator();

    // Create service
    service_ = this->create_service<smooth_nav_msgs::srv::GenerateTrajectory>(
      "~/generate_trajectory",
      std::bind(&TrajectoryGeneratorNode::handleGenerate, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "TrajectoryGeneratorNode ready. Type: %s. Service: ~/generate_trajectory",
      this->get_parameter("generator_type").as_string().c_str());
  }

private:
  std::unique_ptr<smooth_nav_core::ITrajectoryGenerator> generator_;
  rclcpp::Service<smooth_nav_msgs::srv::GenerateTrajectory>::SharedPtr service_;

  void createGenerator()
  {
    std::string type = this->get_parameter("generator_type").as_string();
    double v_max = this->get_parameter("max_velocity").as_double();
    double a_max = this->get_parameter("max_acceleration").as_double();
    double dt = this->get_parameter("time_step").as_double();

    if (type == "constant") {
      auto gen = std::make_unique<smooth_nav_core::ConstantVelocityGenerator>();
      gen->setMaxVelocity(v_max);
      gen->setTimeStep(dt);
      generator_ = std::move(gen);
    } else {
      auto gen = std::make_unique<smooth_nav_core::TrapezoidalVelocityGenerator>();
      gen->setMaxVelocity(v_max);
      gen->setMaxAcceleration(a_max);
      gen->setTimeStep(dt);
      generator_ = std::move(gen);
    }
  }

  void handleGenerate(
    const std::shared_ptr<smooth_nav_msgs::srv::GenerateTrajectory::Request> request,
    std::shared_ptr<smooth_nav_msgs::srv::GenerateTrajectory::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received generate_trajectory request with %zu path points",
      request->smoothed_path.points.size());

    // Convert ROS message to core PathPoint
    std::vector<smooth_nav_core::PathPoint> path;
    for (size_t i = 0; i < request->smoothed_path.points.size(); ++i) {
      smooth_nav_core::PathPoint pt;
      pt.x = request->smoothed_path.points[i].x;
      pt.y = request->smoothed_path.points[i].y;
      if (i < request->smoothed_path.curvatures.size()) {
        pt.curvature = request->smoothed_path.curvatures[i];
      }
      path.push_back(pt);
    }

    // Compute headings
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      path[i].theta = std::atan2(path[i + 1].y - path[i].y,
                                  path[i + 1].x - path[i].x);
    }
    if (path.size() >= 2) {
      path.back().theta = path[path.size() - 2].theta;
    }

    // Override velocity params from request if provided
    if (request->max_velocity > 0.0) {
      auto type = this->get_parameter("generator_type").as_string();
      if (type == "constant") {
        auto * gen = dynamic_cast<smooth_nav_core::ConstantVelocityGenerator *>(generator_.get());
        if (gen) gen->setMaxVelocity(request->max_velocity);
      } else {
        auto * gen = dynamic_cast<smooth_nav_core::TrapezoidalVelocityGenerator *>(generator_.get());
        if (gen) {
          gen->setMaxVelocity(request->max_velocity);
          if (request->max_acceleration > 0.0) {
            gen->setMaxAcceleration(request->max_acceleration);
          }
        }
      }
    }

    try {
      auto traj = generator_->generate(path);

      // Fill response
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

      response->success = true;
      response->message = "Generated " + std::to_string(traj.size()) +
                          " trajectory points using " + generator_->name() +
                          " profile. Duration: " +
                          std::to_string(traj.empty() ? 0.0 : traj.back().timestamp) + " s";

      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("Generation failed: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
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
