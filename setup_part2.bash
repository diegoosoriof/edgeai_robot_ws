# Source from any Bash terminal: source /path/to/edgeai_robot_ws/setup_part2.bash
_EDGEAI_PART2_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(pixi shell-hook --manifest-path "$_EDGEAI_PART2_WS/pixi.toml" --shell bash)" || return 1
if [[ ! -f "$_EDGEAI_PART2_WS/install/setup.bash" ]]; then
    echo "Build first: pixi run colcon build --symlink-install"
    unset _EDGEAI_PART2_WS
    return 1
fi
source "$_EDGEAI_PART2_WS/install/setup.bash"
export CYCLONEDDS_URI="file://$_EDGEAI_PART2_WS/src/edgeai_rescue/config/local_dds.xml"
unset _EDGEAI_PART2_WS
