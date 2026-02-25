#!/bin/bash
# ──────────────────────────────────────────────────────────────
# setup_workspace.sh — Build the workspace inside Docker
# Usage: ./scripts/setup_workspace.sh
# ──────────────────────────────────────────────────────────────
set -euo pipefail

echo "=== smooth_nav workspace setup ==="

# Source ROS 2
source /opt/ros/humble/setup.bash

cd /ros2_ws

echo ">>> Building all packages..."
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo ">>> Sourcing workspace..."
source install/setup.bash

echo ">>> Running unit tests..."
colcon test --packages-select smooth_nav_core
colcon test-result --verbose

echo ""
echo "=== Workspace ready! ==="
echo "  Launch demo: ros2 launch smooth_nav_bringup smooth_nav.launch.py"
echo "  Run tests:   colcon test --return-code-on-test-failure"
