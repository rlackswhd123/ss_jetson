#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ss_robot_ws}"
SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-$WORKSPACE_DIR/config/lidar_only_slam.yaml}"
ODOM_COMMAND="${ODOM_COMMAND:-ros2 run ros2_laser_scan_matcher laser_scan_matcher --ros-args -p publish_odom:=/odom -p publish_tf:=true}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${SS_DEPTH_STATE_DIR:-/tmp/ss_depth_dashboard}"

SETUP_COMMAND="source /opt/ros/jazzy/setup.bash && source \"$WORKSPACE_DIR/install/setup.bash\""

open_terminal() {
  local title="$1"
  local command="$2"
  local full_command

  full_command=$(
    printf "SS_DEPTH_TITLE=%q SS_DEPTH_STATE_DIR=%q SS_DEPTH_SETUP_COMMAND=%q SS_DEPTH_COMMAND=%q %q" \
      "$title" \
      "$STATE_DIR" \
      "$SETUP_COMMAND" \
      "$command" \
      "$SCRIPT_DIR/terminal_runner.sh"
  )

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$full_command"
    return
  fi

  if command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -T "$title" -e bash -lc "$full_command"
    return
  fi

  echo "No supported terminal emulator found. Install gnome-terminal or run commands manually." >&2
  exit 1
}

mkdir -p "$STATE_DIR"
rm -f "$STATE_DIR/pids"

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  echo "Missing workspace setup: $WORKSPACE_DIR/install/setup.bash" >&2
  echo "Build the workspace first:" >&2
  echo "  cd $WORKSPACE_DIR" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  echo "  colcon build --symlink-install" >&2
  exit 1
fi

if ! bash -lc "$SETUP_COMMAND && ros2 pkg executables ros2_laser_scan_matcher | grep -q '^ros2_laser_scan_matcher laser_scan_matcher$'"; then
  echo "Missing executable: ros2_laser_scan_matcher laser_scan_matcher" >&2
  echo "Build it first:" >&2
  echo "  cd $WORKSPACE_DIR" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  echo "  colcon build --symlink-install --packages-select csm ros2_laser_scan_matcher ss_depth" >&2
  exit 1
fi

if [ ! -f "$SLAM_PARAMS_FILE" ]; then
  echo "Missing SLAM params file: $SLAM_PARAMS_FILE" >&2
  exit 1
fi

open_terminal "ss_depth lidar" \
  "ros2 launch sllidar_ros2 sllidar_a2m7_launch.py"

sleep 2

open_terminal "ss_depth laser tf" \
  "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser"

sleep 1

open_terminal "ss_depth odom" \
  "$ODOM_COMMAND"

sleep 4

open_terminal "ss_depth slam" \
  "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$SLAM_PARAMS_FILE"

sleep 3

open_terminal "ss_depth realsense" \
  "ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true"

sleep 2

open_terminal "ss_depth dashboard" \
  "ros2 run ss_depth dashboard_node"

echo "Started ss_depth terminals."
