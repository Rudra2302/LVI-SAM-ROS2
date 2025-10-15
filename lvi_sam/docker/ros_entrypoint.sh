#!/bin/bash

set -e

source "/opt/ros/humble/setup.bash"

echo "==============ROS2 Docker Env Ready================"
cd /home/ros2_ws

exec "$@"
