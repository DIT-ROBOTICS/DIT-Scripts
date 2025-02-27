#!/bin/bash

if [ "$ROS_DISTRO" = 'noetic' ]; then
    source /opt/ros/noetic/setup.bash
    roslaunch foxglove_bridge foxglove_bridge.launch;
elif [ "$ROS_DISTRO" = 'humble' ]; then
    source /opt/ros/humble/setup.bash
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' &
    ros2 run image_tools cam2image --ros-args --log-level WARN -p video_device:=/dev/video0;
fi

exec "$@"