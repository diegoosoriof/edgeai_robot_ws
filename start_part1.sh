#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"
source "$WS_DIR/setup_workspace.bash"
export ROS_LOG_DIR="$WS_DIR/log/part1"
exec ros2 launch edgeai_robot_description part1.launch.py "$@"
