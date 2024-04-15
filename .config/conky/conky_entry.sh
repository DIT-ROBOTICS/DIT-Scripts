#!/bin/bash

DIR="/home/ditrobotics/.config/conky"

conky -c $DIR/conky-esp-config/.conky_esp32 &
$DIR/lean-conky-config/start-lcc.sh
$DIR/conky-esp-config/battery_watchdog.sh &

# Web
/home/ditrobotics/DIT-Scripts/web-ui/scripts/aio.sh &

while ! nc -z localhost 5000; do   
  sleep 0.1
done

firefox http://localhost:5000 &
