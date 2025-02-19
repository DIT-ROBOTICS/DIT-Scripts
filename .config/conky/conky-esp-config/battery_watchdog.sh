#!/bin/bash

DIR="/home/ditrobotics/DIT-Scripts/.config/conky/conky-esp-config"
pactl set-sink-volume @DEFAULT_SINK@ 95%
cd $DIR
sleep 1

# Boot-up sound effect
ffplay -nodisp -autoexit welcome.mp3

while true; do
  # voltage=$(./conky_esp32.sh)

  if (( $(echo "$voltage < 17.5" | bc -l) )); then
    if ! pgrep -x "ffplay" > /dev/null; then
      
      # Broadcast shutdown message
      wall < shutdown_message.txt
      # Brocast low battery warning
      ffplay -nodisp -autoexit warning_low_battery.mp3 > /dev/null 2>&1 &
      # ffplay -nodisp -autoexit mario_death.mp3 > /dev/null 2>&1 && ffplay -nodisp -autoexit warning_low_battery.mp3 > /dev/null 2>&1 &
      
    fi
  fi

  sleep 5
done

