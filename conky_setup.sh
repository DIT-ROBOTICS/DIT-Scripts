#!/bin/bash

echo "Copying conky config files to ~/.config/conky/"
cp -r .config/conky /home/ditrobotics/.config/
echo "Copying autostart config files to ~/.config/autostart/"
cp -r .config/autostart /home/ditrobotics/.config/
