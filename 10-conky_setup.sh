#!/bin/bash

# Setup conky layout (including relative scripts)
echo "Copying conky config files to ~/.config/conky/"
cp -r .config/conky /home/ditrobotics/.config/

# Setup autostart script for conky_entry.sh
echo "Copying autostart config files to ~/.config/autostart/"
cp -r .config/autostart /home/ditrobotics/.config/

# Setup udev rules for USB devices (e.g. esp32|stm32|imu|lidar|vive|touchscreen|...)
echo "Copying udev rules files to /etc/udev/rules.d/"
cp -r udev/. /etc/udev/rules.d/.

# Reload udev rules
udevadm control --reload-rules && udevadm trigger
