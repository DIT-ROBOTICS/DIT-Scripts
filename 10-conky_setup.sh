#!/bin/bash

# Setup conky layout (including relative scripts)
echo "Copying conky config files to ~/.config/conky/"
cp -r .config/conky /home/vision/.config/

# Setup autostart script for conky_entry.sh
echo "Copying autostart config files to ~/.config/autostart/"
cp -r .config/autostart /home/vision/.config/
