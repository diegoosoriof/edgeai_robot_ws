#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"
export PATH="$WS_DIR/.pixi/envs/default/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$WS_DIR/src/edgeai_robot_description/config/rescue_simulation_dds.xml"
export ROS_LOG_DIR="$WS_DIR/log/part1" ROS_HOME="$WS_DIR/.ros"
source install/setup.bash
exec ros2 launch edgeai_robot_description part1.launch.py "$@"
