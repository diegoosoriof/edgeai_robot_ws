#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"
source "$WS_DIR/setup_workspace.bash"
export ROS_LOG_DIR="$WS_DIR/log/part2"
exec ros2 launch edgeai_rescue rescue_part2.launch.py "$@"
