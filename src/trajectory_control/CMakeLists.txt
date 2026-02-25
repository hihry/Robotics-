cmake_minimum_required(VERSION 3.8)
project(trajectory_control)

# ---- Compiler Settings ----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Find Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Include Directories ----
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ============================================================================
# LIBRARIES (modular — each algorithm is its own library)
# ============================================================================

# Path Smoother Library
add_library(path_smoother_lib
  src/path_smoother.cpp
)
ament_target_dependencies(path_smoother_lib
  rclcpp nav_msgs geometry_msgs visualization_msgs
)
target_include_directories(path_smoother_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Trajectory Generator Library
add_library(trajectory_generator_lib
  src/trajectory_generator.cpp
)
ament_target_dependencies(trajectory_generator_lib
  rclcpp nav_msgs geometry_msgs
)
target_include_directories(trajectory_generator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Trajectory Controller Library
add_library(trajectory_controller_lib
  src/trajectory_controller.cpp
)
ament_target_dependencies(trajectory_controller_lib
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_include_directories(trajectory_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================================
# EXECUTABLES
# ============================================================================

# Main Trajectory Node (ROS2 node that ties everything together)
add_executable(trajectory_node
  src/trajectory_node.cpp
)
target_link_libraries(trajectory_node
  path_smoother_lib
  trajectory_generator_lib
  trajectory_controller_lib
)
ament_target_dependencies(trajectory_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

# ============================================================================
# INSTALL
# ============================================================================

# Install libraries
install(TARGETS
  path_smoother_lib
  trajectory_generator_lib
  trajectory_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts
install(PROGRAMS
  scripts/plot_trajectory.py
  scripts/waypoint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# ============================================================================
# TESTING
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Unit Tests
  ament_add_gtest(test_path_smoother test/test_path_smoother.cpp)
  target_link_libraries(test_path_smoother path_smoother_lib)

  ament_add_gtest(test_trajectory_generator test/test_trajectory_generator.cpp)
  target_link_libraries(test_trajectory_generator trajectory_generator_lib)

  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller trajectory_controller_lib)

  # Lint (optional — comment out if too strict)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
