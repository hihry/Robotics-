cmake_minimum_required(VERSION 3.14)
project(smooth_nav_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smooth_nav_core REQUIRED)
find_package(smooth_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ─── Path Smoother Service Node ──────────────────────────────────────────
add_executable(path_smoother_node
  src/path_smoother_node.cpp
)
ament_target_dependencies(path_smoother_node
  rclcpp smooth_nav_core smooth_nav_msgs
  nav_msgs geometry_msgs visualization_msgs
)

# ─── Trajectory Generator Service Node ───────────────────────────────────
add_executable(trajectory_generator_node
  src/trajectory_generator_node.cpp
)
ament_target_dependencies(trajectory_generator_node
  rclcpp smooth_nav_core smooth_nav_msgs
  nav_msgs geometry_msgs
)

# ─── Install ─────────────────────────────────────────────────────────────
install(TARGETS
  path_smoother_node
  trajectory_generator_node
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

# ─── Testing ─────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
