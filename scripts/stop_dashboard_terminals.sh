#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="${SS_DEPTH_STATE_DIR:-/tmp/ss_depth_dashboard}"
PID_FILE="$STATE_DIR/pids"

stop_recorded_terminals() {
  if [ ! -f "$PID_FILE" ]; then
    return
  fi

  awk '{print $1}' "$PID_FILE" | while read -r pid; do
    if [ -n "$pid" ] && kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1 || true
    fi
  done

  sleep 2

  awk '{print $1}' "$PID_FILE" | while read -r pid; do
    if [ -n "$pid" ] && kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done

  rm -f "$PID_FILE"
}

stop_matching_processes() {
  pkill -INT -f "ros2 launch sllidar_ros2 sllidar_a2m7_launch.py" >/dev/null 2>&1 || true
  pkill -INT -f "static_transform_publisher .*base_link laser" >/dev/null 2>&1 || true
  pkill -INT -f "ros2 run ros2_laser_scan_matcher laser_scan_matcher" >/dev/null 2>&1 || true
  pkill -INT -f "ros2 launch slam_toolbox online_async_launch.py" >/dev/null 2>&1 || true
  pkill -INT -f "ros2 launch realsense2_camera rs_launch.py" >/dev/null 2>&1 || true
  pkill -INT -f "ros2 run ss_depth dashboard_node" >/dev/null 2>&1 || true
}

stop_recorded_terminals
stop_matching_processes

echo "Stopped ss_depth terminals."
