#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
