#!/bin/bash

DIR="/home/ditrobotics/.config/conky"

conky -c $DIR/conky-esp-config/.conky_esp32 &
$DIR/lean-conky-config/start-lcc.sh;

# Set the volume to 95%
sleep 3
pactl set-sink-volume @DEFAULT_SINK@ 95%
# Boot-up sound effect
sleep 1
ffplay -nodisp -autoexit welcome.mp3
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
