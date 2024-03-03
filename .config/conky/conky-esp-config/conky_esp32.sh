#!/bin/bash

tmp=$(timeout 0.2s cat /dev/esp32 | grep --line-buffered 'batteryVoltage' | awk -F':' '{print $2}')
echo $tmp | awk '{print $NF}'
