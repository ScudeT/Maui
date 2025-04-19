#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/jazzy/
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash

echo "Provided arguments: $@"

# Change directory to your ROS2 workspace
cd /home/ros/ros2_ws

# If no arguments are provided, open an interactive bash terminal.
if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi
