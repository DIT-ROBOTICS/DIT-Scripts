#!/bin/bash

cd ~/robot_status_br
. /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-skip micro_ros_msgs

source ~/robot_status_br/install/setup.bash
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
# ros2 run robot_status_monitor robot_status_node
ros2 launch robot_status_monitor robot_status.launch

while true; do
    sleep 60
done

exec "$@"