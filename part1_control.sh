#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"
# Use the already installed Pixi environment. This also works when an older
# pixi binary cannot read the repository's newer lockfile.
export PATH="$WS_DIR/.pixi/envs/default/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$WS_DIR/src/edgeai_robot_description/config/rescue_simulation_dds.xml"
export ROS_LOG_DIR="$WS_DIR/log/part1" ROS_HOME="$WS_DIR/.ros"
set +u
source install/setup.bash
set -u
case "${1:-}" in
  start)
    echo 'Waiting for simulated robot odometry/TF...'
    timeout 120 ros2 topic echo /tf --once >/dev/null
    ros2 lifecycle set /robot1/slam_toolbox configure
    ros2 lifecycle set /robot1/slam_toolbox activate
    echo 'Starting Nav2 on the new live map...'
    ros2 service call /lifecycle_manager_navigation/manage_nodes \
      nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
    ros2 service call /part1/start std_srvs/srv/Trigger '{}'
    ;;
  save)
    RUN_DIR="$(cat "${PART1_OUTPUT_ROOT:-runs/part1}/latest_run.txt")"
    if [[ -z "$RUN_DIR" || ! -d "$RUN_DIR" ]]; then
      echo 'No current run directory. Start first.' >&2
      exit 1
    fi
    ros2 run nav2_map_server map_saver_cli -f "$RUN_DIR/map" \
      --ros-args -p use_sim_time:=true -r map:=/map
    ;;
  status)
    ros2 lifecycle get /robot1/slam_toolbox
    ros2 topic echo /part1/summary --once
    ;;
  *)
    echo "Usage: $0 {start|save|status}" >&2
    exit 2
    ;;
esac
