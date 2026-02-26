cmake_minimum_required(VERSION 3.14)
project(smooth_nav_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── Install Python package (waypoint_client, safety_watchdog) ────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/waypoint_client_node.py
  ${PROJECT_NAME}/safety_watchdog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
