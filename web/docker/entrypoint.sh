#!/bin/bash

# Set ROS_DOMAIN_ID based on the hostname
export ROS_DOMAIN_ID=${HOSTNAME##*-}
echo "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"

if [ "$ROS_DISTRO" = 'noetic' ]; then
    roslaunch foxglove_bridge foxglove_bridge.launch;
elif [ "$ROS_DISTRO" = 'humble' ]; then
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' &
    ros2 run image_tools cam2image --ros-args --log-level WARN -p video_device:=/dev/video0;
fi

exec "$@"