#!/bin/bash

DIR="/home/ditrobotics/.config/conky"

conky -c $DIR/conky-esp-config/.conky_esp32 &
$DIR/lean-conky-config/start-lcc.sh;

sleep 3
# Set the volume to 95%
VOLUME_LEVEL="95%"
UBUNTU_VERSION=$(lsb_release -r | awk '{print $2}')

if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    pactl set-sink-volume @DEFAULT_SINK@ "$VOLUME_LEVEL"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    amixer set Master "$VOLUME_LEVEL"
else
    echo "Unrecognized Ubuntu version, trying to auto-detect available tools..."
    if command -v pactl &> /dev/null; then
        pactl set-sink-volume @DEFAULT_SINK@ "$VOLUME_LEVEL"
    elif command -v amixer &> /dev/null; then
        amixer set Master "$VOLUME_LEVEL"
    else
        echo "Unable to find volume control tool! Please check system installation."
    fi
fi

# Boot-up sound effect
sleep 1
ffplay -nodisp -autoexit /home/ditrobotics/DIT-Scripts/.config/conky/conky-esp-config/welcome.mp3
# ========================================
# ===== PUT SOUND EFFECT MODULE HERE =====

$DIR/conky-esp-config/10-battery_watchdog/battery_watchdog.sh &
$DIR/conky-esp-config/10-network_watchdog/network_watchdog.sh &

# ========================================

# # Run the Web Pannel essential scripts
# /home/ditrobotics/DIT-Scripts/web-ui/scripts/aio.sh &
# # Wait for the web server to start
# while ! nc -z localhost 5000; do   
#   sleep 0.1
# done
# 
# # Open Robot UI in Firefox
# firefox http://localhost:5000 http://${HOSTNAME}-esp.local &
