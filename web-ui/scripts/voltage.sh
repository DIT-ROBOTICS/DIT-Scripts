#!/bin/bash

while true; do
    output=$(/home/ditrobotics/.config/conky/conky-esp-config/conky_esp32.sh)
    echo $output > /home/share/data/voltage.cache 
    sleep 1
done
