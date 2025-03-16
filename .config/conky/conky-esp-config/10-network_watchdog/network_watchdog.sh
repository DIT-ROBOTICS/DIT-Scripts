#!/bin/bash

DIR="/home/ditrobotics/DIT-Scripts/.config/conky/conky-esp-config/10-network_watchdog"
cd $DIR

# https://ttsmaker.com/
# 2599 - Christ-United States Male

prev_state=0
while true; do
      
  # Check Wi-Fi connection status
  state=$(nmcli -t -f GENERAL.STATE device show wlp2s0 | awk -F: '{print $2}' | awk '{print $1}')
  
  if [[ $state -eq 100 ]]; then
      # Wi-Fi is connected
      if [[ $prev_state -ne 100 ]]; then
          ffplay -nodisp -autoexit starlink_engaged.mp3 > /dev/null 2>&1 &
      fi
      echo -e "\e[32mWi-Fi is connected\e[0m"
  else
      # Wi-Fi is disconnected
      if [[ $prev_state -eq 100 ]]; then
          ffplay -nodisp -autoexit wifi_problem.mp3 > /dev/null 2>&1 &
      fi
      echo -e "\e[33mWi-Fi is not connected [$(nmcli -t -f GENERAL.STATE device show wlp2s0)]\e[0m"
  fi
  prev_state=$state

  sleep 5
done

