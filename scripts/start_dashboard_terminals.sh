#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ss_robot_ws}"
SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-$WORKSPACE_DIR/config/lidar_only_slam.yaml}"
ODOM_COMMAND="${ODOM_COMMAND:-ros2 run ros2_laser_scan_matcher laser_scan_matcher --ros-args -p publish_odom:=/odom -p publish_tf:=true}"

SETUP_COMMAND="source /opt/ros/jazzy/setup.bash && source \"$WORKSPACE_DIR/install/setup.bash\""

open_terminal() {
  local title="$1"
  local command="$2"
  local full_command="$SETUP_COMMAND && $command; exec bash"

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

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  echo "Missing workspace setup: $WORKSPACE_DIR/install/setup.bash" >&2
  echo "Build the workspace first: cd $WORKSPACE_DIR && colcon build --packages-select ss_depth" >&2
  exit 1
fi

open_terminal "ss_depth realsense" \
  "ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true"

open_terminal "ss_depth lidar" \
  "ros2 launch sllidar_ros2 sllidar_a2m7_launch.py"

open_terminal "ss_depth laser tf" \
  "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser"

open_terminal "ss_depth odom" \
  "$ODOM_COMMAND"

open_terminal "ss_depth slam" \
  "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$SLAM_PARAMS_FILE"

open_terminal "ss_depth dashboard" \
  "ros2 run ss_depth dashboard_node"

echo "Started ss_depth terminals."
