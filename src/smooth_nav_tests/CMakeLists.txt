cmake_minimum_required(VERSION 3.14)
project(smooth_nav_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(smooth_nav_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Integration test: path smoother service
  ament_add_gtest(test_smoother_service
    test/integration/test_smoother_service.cpp
    TIMEOUT 30
  )
  ament_target_dependencies(test_smoother_service
    rclcpp
    smooth_nav_msgs
    geometry_msgs
  )

  # Integration test: trajectory generator service
  ament_add_gtest(test_generator_service
    test/integration/test_generator_service.cpp
    TIMEOUT 30
  )
  ament_target_dependencies(test_generator_service
    rclcpp
    smooth_nav_msgs
    geometry_msgs
  )

  # Integration test: trajectory tracker action
  ament_add_gtest(test_tracker_action
    test/integration/test_tracker_action.cpp
    TIMEOUT 60
  )
  ament_target_dependencies(test_tracker_action
    rclcpp
    rclcpp_action
    smooth_nav_msgs
    nav_msgs
    geometry_msgs
  )

  # System launch test
  add_launch_test(
    test/system/test_full_pipeline.launch.py
    TIMEOUT 120
  )
endif()

ament_package()
