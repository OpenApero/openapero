#!/usr/bin/env bash
# Open RViz on the host, attached to the ROS master of a running Open Apero container. Usage:
#   docker/rviz_host.sh [RVIZ_CONFIG]     # default: open_apero_ros/rviz/experiment.rviz
# Start the experiment in the container with rviz:=false first. docker/run.sh shares the host network, so the
# container's master is the host's localhost:11311 (or $ROS_MASTER_URI). Needs ROS Noetic and this workspace built
# on the host: RViz loads voxblox_rviz_plugin, jsk_rviz_plugins and the meshes of open_apero_ros. The end-of-run
# RViz screenshot is not saved: its path is inside the container.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The workspace this checkout sits in, <ws>/src/open_apero/docker, unless OPEN_APERO_WS_SETUP names another.
ws_setup="${OPEN_APERO_WS_SETUP:-$(cd "$script_dir/../../.." && pwd)/devel/setup.bash}"
if [ ! -f "$ws_setup" ]; then
  echo "error: $ws_setup not found; build the workspace on the host or set OPEN_APERO_WS_SETUP" >&2
  exit 1
fi

# setup.bash reads unset variables.
set +u
source /opt/ros/noetic/setup.bash
source "$ws_setup"
set -u

config="${1:-$(rospack find open_apero_ros)/rviz/experiment.rviz}"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
# Without it, RViz opens a modal end-of-life dialog and ignores Ctrl+C.
export DISABLE_ROS1_EOL_WARNINGS=1
exec rviz -d "$config"
