#!/bin/bash

DIR="/home/ditrobotics/.config/conky"

conky -c $DIR/conky-esp-config/.conky_esp32 &
$DIR/lean-conky-config/start-lcc.sh
$DIR/conky-esp-config/battery_watchdog.sh &
