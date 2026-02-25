cmake_minimum_required(VERSION 3.14)
project(smooth_nav_core VERSION 1.0.0 LANGUAGES CXX)

# ─── C++ Standard ────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── Options ─────────────────────────────────────────────────────────────────
option(BUILD_TESTING "Build unit tests" ON)

# ─── Library target ──────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}
  src/path_smoother/cubic_spline_smoother.cpp
  src/path_smoother/bspline_smoother.cpp
  src/trajectory_generator/trapezoidal_velocity_generator.cpp
  src/trajectory_generator/constant_velocity_generator.cpp
  src/controller/pure_pursuit_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Strict warnings
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic>
  $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic>
)

# ─── ament_cmake integration (optional — only if found) ─────────────────────
# This allows the package to be both a standalone CMake lib AND a ROS 2 package.
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  # Export as an ament package so smooth_nav_ros etc. can find_package(smooth_nav_core)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # ─── Tests ───────────────────────────────────────────────────────────────
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest QUIET)
    if(ament_cmake_gtest_FOUND)
      ament_add_gtest(test_cubic_spline test/unit/test_cubic_spline.cpp)
      target_link_libraries(test_cubic_spline ${PROJECT_NAME})

      ament_add_gtest(test_trapezoidal_velocity test/unit/test_trapezoidal_velocity.cpp)
      target_link_libraries(test_trapezoidal_velocity ${PROJECT_NAME})

      ament_add_gtest(test_pure_pursuit test/unit/test_pure_pursuit.cpp)
      target_link_libraries(test_pure_pursuit ${PROJECT_NAME})

      ament_add_gtest(test_pid_controller test/unit/test_pid_controller.cpp)
      target_link_libraries(test_pid_controller ${PROJECT_NAME})

      ament_add_gtest(test_geometry_utils test/unit/test_geometry_utils.cpp)
      target_link_libraries(test_geometry_utils ${PROJECT_NAME})
    endif()
  endif()

  ament_package()
else()
  # ─── Standalone CMake install (no ROS 2) ─────────────────────────────────
  include(GNUInstallDirs)

  install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

  # ─── Tests (standalone GTest) ────────────────────────────────────────────
  if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
      enable_testing()

      function(add_core_test test_name test_src)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
        add_test(NAME ${test_name} COMMAND ${test_name})
      endfunction()

      add_core_test(test_cubic_spline      test/unit/test_cubic_spline.cpp)
      add_core_test(test_trapezoidal_velocity test/unit/test_trapezoidal_velocity.cpp)
      add_core_test(test_pure_pursuit       test/unit/test_pure_pursuit.cpp)
      add_core_test(test_pid_controller     test/unit/test_pid_controller.cpp)
      add_core_test(test_geometry_utils     test/unit/test_geometry_utils.cpp)
    endif()
  endif()
endif()
