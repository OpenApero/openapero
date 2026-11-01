#!/usr/bin/env bash
# Source ROS and the built Open Apero workspace, then run the command (default: bash).
# The trailing `--` keeps setup.bash from parsing this script's arguments as its own.
set -e
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash --
# shellcheck source=/dev/null
source "${WS:-/root/catkin_ws}/devel/setup.bash" --
exec "$@"
