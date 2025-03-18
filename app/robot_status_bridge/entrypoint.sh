#!/bin/bash

cd ~/robot_status_br
# . /opt/ros/humble/setup.sh && colcon build --symlink-install

# source ~/robot_status_br/install/setup.bash
# ros2 run robot_status_monitor robot_status_publisher

while true; do
    sleep 60
done

exec "$@"