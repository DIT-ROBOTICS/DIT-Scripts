#!/bin/bash

PORT="/dev/ttyACM0"
BAUD=115200
stty -F $PORT $BAUD

cat < $PORT

