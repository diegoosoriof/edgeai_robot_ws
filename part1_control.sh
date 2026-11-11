#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"
source "$WS_DIR/setup_workspace.bash"
export ROS_LOG_DIR="$WS_DIR/log/part1"
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
    python - "$RUN_DIR/map.yaml" <<'PY'
from pathlib import Path
import sys

import cv2
import yaml

metadata_path = Path(sys.argv[1])
image_path = Path(yaml.safe_load(metadata_path.read_text())['image'])
if not image_path.is_absolute():
    image_path = metadata_path.parent / image_path
image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
if image is None:
    raise SystemExit(f'Could not read saved map image: {image_path}')
output = metadata_path.with_suffix('.png')
if not cv2.imwrite(str(output), image):
    raise SystemExit(f'Could not write map PNG: {output}')
print(f'Map image saved: {output}')
PY
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
