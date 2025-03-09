#!/bin/bash

# sudo chmod a+rw /dev/snd/*

cd ~/
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip -d ~/vosk_model

cd ~/voice_control_ws
. /opt/ros/humble/setup.sh && colcon build --symlink-install

source ~/voice_control_ws/install/setup.bash
ros2 run voice_nav speech_to_nav
# sleep 3600

exec "$@"