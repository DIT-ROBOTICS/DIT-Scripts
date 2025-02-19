#!/bin/bash

DIR="/home/ditrobotics/.config/conky"

conky -c $DIR/conky-esp-config/.conky_esp32 &
$DIR/lean-conky-config/start-lcc.sh;
$DIR/conky-esp-config/battery_watchdog.sh &

# # Run the Web Pannel essential scripts
# /home/ditrobotics/DIT-Scripts/web-ui/scripts/aio.sh &
# # Wait for the web server to start
# while ! nc -z localhost 5000; do   
#   sleep 0.1
# done
# 
# # Open Robot UI in Firefox
# firefox http://localhost:5000 http://${HOSTNAME}-esp.local &
