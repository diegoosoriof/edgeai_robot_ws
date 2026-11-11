# Source from Bash: source /path/to/edgeai_robot_ws/setup_workspace.bash
_EDGEAI_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$_EDGEAI_WS/.pixi/envs/default/bin/ros2" || ! -f "$_EDGEAI_WS/install/setup.bash" ]]; then
    echo 'Install and build first: pixi install && pixi run colcon build --symlink-install' >&2
    unset _EDGEAI_WS
    return 1
fi
export PATH="$_EDGEAI_WS/.pixi/envs/default/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$_EDGEAI_WS/src/edgeai_robot_description/config/rescue_simulation_dds.xml"
export ROS_LOG_DIR="$_EDGEAI_WS/log/runtime" ROS_HOME="$_EDGEAI_WS/.ros"
# Generated ROS setup scripts do not support Bash nounset.
_EDGEAI_NOUNSET=false
if [[ $- == *u* ]]; then _EDGEAI_NOUNSET=true; set +u; fi
source "$_EDGEAI_WS/install/setup.bash"
if $_EDGEAI_NOUNSET; then set -u; fi
unset _EDGEAI_WS _EDGEAI_NOUNSET
