#!/bin/bash
# ──────────────────────────────────────────────────────────────
# run_tests.sh — Run all tests (unit + integration + system)
# Usage: ./scripts/run_tests.sh [package_name]
# ──────────────────────────────────────────────────────────────
set -euo pipefail

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

PACKAGES=${1:-"smooth_nav_core smooth_nav_tests"}

echo "=== Running tests for: ${PACKAGES} ==="

colcon test \
  --packages-select ${PACKAGES} \
  --return-code-on-test-failure \
  --event-handlers console_cohesion+

echo ""
echo "=== Test Results ==="
colcon test-result --verbose
