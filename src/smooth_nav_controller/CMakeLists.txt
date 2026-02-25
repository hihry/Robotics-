cmake_minimum_required(VERSION 3.14)
project(smooth_nav_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(smooth_nav_core REQUIRED)
find_package(smooth_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ─── Trajectory Tracker Node ─────────────────────────────────────────────
add_executable(trajectory_tracker_node
  src/trajectory_tracker_node.cpp
)
ament_target_dependencies(trajectory_tracker_node
  rclcpp rclcpp_action
  smooth_nav_core smooth_nav_msgs
  nav_msgs geometry_msgs std_msgs
  tf2 tf2_geometry_msgs
)

# ─── Install ─────────────────────────────────────────────────────────────
install(TARGETS
  trajectory_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
