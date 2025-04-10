#!/bin/bash

DIR="/home/ditrobotics/DIT-Scripts/.config/conky/conky-esp-config/10-battery_watchdog"
cd $DIR

BATTERY_STATUS_FILE="/home/ditrobotics/DIT-Scripts/app/robot_status_bridge/robot_status_br/tmp/battery_status.json"
LOW_BATTERY_THRESHOLD=17.5
DISCONNECTED_THRESHOLD=0.0
LOW_BATTERY_SOUND="warning_low_battery_en_1.mp3"
DISCONNECTED_SOUND="battery_removed.mp3"
DISCONNECTED_PLAYED=false

while true; do
  if [ -f "$BATTERY_STATUS_FILE" ]; then
    voltage=$(jq -r '.voltage' "$BATTERY_STATUS_FILE")

    if (( $(echo "$voltage == $DISCONNECTED_THRESHOLD" | bc -l) )); then
      if [ "$DISCONNECTED_PLAYED" = false ]; then
        if ! pgrep -x "ffplay" > /dev/null; then
          # Play battery disconnected sound once
          ffplay -nodisp -autoexit "$DISCONNECTED_SOUND" > /dev/null 2>&1 &
          DISCONNECTED_PLAYED=true
        fi
      fi
    else
      DISCONNECTED_PLAYED=false

      if (( $(echo "$voltage < $LOW_BATTERY_THRESHOLD" | bc -l) )); then
        if ! pgrep -x "ffplay" > /dev/null; then
          # Broadcast low battery warning
          ffplay -nodisp -autoexit "$LOW_BATTERY_SOUND" > /dev/null 2>&1 &
        fi
      fi
    fi
  fi

  sleep 5
done

