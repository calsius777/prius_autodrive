#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash 2>/dev/null || true
exec python3 /colcon_ws/tools/sensor_snapshot_tool/capture_six_cam_lidar.py "$@"
