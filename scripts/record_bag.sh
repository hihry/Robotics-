#!/bin/bash
# ──────────────────────────────────────────────────────────────
# record_bag.sh — Record a rosbag of the smooth_nav demo
# Usage: ./scripts/record_bag.sh [output_dir]
# ──────────────────────────────────────────────────────────────
set -euo pipefail

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

OUTPUT_DIR=${1:-"/ros2_ws/bags/smooth_nav_$(date +%Y%m%d_%H%M%S)"}

echo "=== Recording rosbag to: ${OUTPUT_DIR} ==="
echo "Press Ctrl+C to stop recording."
echo ""

ros2 bag record \
  /cmd_vel \
  /odom \
  /smoothed_path \
  /original_waypoints \
  /actual_path \
  /tracking_error \
  /controller_diagnostics \
  /tf \
  /tf_static \
  -o "${OUTPUT_DIR}"
