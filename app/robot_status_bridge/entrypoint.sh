#!/bin/bash

# sudo chmod a+rw /dev/snd/*

cd ~/robot_status_br
# . /opt/ros/humble/setup.sh && colcon build --symlink-install

# source ~/robot_status_br/install/setup.bash
# ros2 run voice_nav speech_to_nav
    #   bash -c "source /opt/ros/humble/setup.bash &&
    #            source /root/ros2_ws/install/setup.bash &&
    #            ros2 run battery_monitor battery_publisher"

while true; do
    sleep 60
done

exec "$@"