#!/bin/bash

DIR="/home/ditrobotics/DIT-Scripts/.config/conky/conky-esp-config"
pactl set-sink-volume @DEFAULT_SINK@ 60%
cd $DIR
sleep 1
ffplay -nodisp -autoexit welcome.mp3

while true; do
  voltage=$(./conky_esp32.sh)

  if (( $(echo "$voltage < 17.5" | bc -l) )); then
    if ! pgrep -x "ffplay" > /dev/null; then
      # ffplay -nodisp -autoexit mario_death.mp3 > /dev/null 2>&1 && ffplay -nodisp -autoexit warning_low_battery.mp3 > /dev/null 2>&1 &
      wall < shutdown_message.txt
      ffplay -nodisp -autoexit warning_low_battery.mp3 > /dev/null 2>&1 &
    fi
  fi

  sleep 5
done

