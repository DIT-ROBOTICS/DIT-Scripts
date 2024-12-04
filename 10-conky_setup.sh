#!/bin/bash

echo "Copying conky config files to ~/.config/conky/"
cp -r .config/conky /home/ditrobotics/.config/
echo "Copying autostart config files to ~/.config/autostart/"
cp -r .config/autostart /home/ditrobotics/.config/
echo "Copying udev rules files to /etc/udev/rules.d/"
cp -r udev/. /etc/udev/rules.d/.

udevadm control --reload-rules && udevadm trigger
