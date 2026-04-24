#!/bin/bash

if [ "$ROS_DISTRO" = 'noetic' ]; then
    source /opt/ros/noetic/setup.bash
    roslaunch foxglove_bridge foxglove_bridge.launch;
elif [ "$ROS_DISTRO" = 'humble' ]; then
    source /opt/ros/humble/setup.bash
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
    topics_glob="['/robot_status/battery_voltage', '/robot_status/usb/*', '/robot/startup/plug', '/robot/startup/groups_state', '/robot/startup/game_time', '/game_score', '/score', '/robot/startup/ideal_score', '/sima_*/status', '/robot/on_take']"
    services_glob="['/robot/startup/ready_signal', '/robot/startup/web_plan']"
    params_glob="['rival_inscribed_radius', 'dock_rival_radius', 'dock_rival_degree', 'pantry_aggressiveness', 'pantry_sensitivity', 'pantry_rival_sigma', 'pantry_rival_distance_threshold', 'collection_aggressiveness', 'collection_sensitivity', 'collection_rival_sigma', 'collection_rival_distance_threshold', 'flip_distance_threshold', 'cursor_tolerance', 'sima_start_time', 'plan_code']"
    actions_glob="['__blocked__']"
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml "topics_glob:=$topics_glob" "services_glob:=$services_glob" "params_glob:=$params_glob" "actions_glob:=$actions_glob" &
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox joy_vel:=cmd_vel_nav;
    # ros2 run image_tools cam2image --ros-args --log-level WARN -p video_device:=/dev/video0;
fi

exec "$@"