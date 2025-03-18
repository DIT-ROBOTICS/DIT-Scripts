#!/bin/bash

WS_DIR=~/esp_daemon

cd WS_DIR
# . /opt/ros/humble/setup.sh && colcon build --symlink-install

source $WS_DIR/install/setup.bash
# ros2 run robot_status_monitor robot_status_node

while true; do
    sleep 60
done

exec "$@"