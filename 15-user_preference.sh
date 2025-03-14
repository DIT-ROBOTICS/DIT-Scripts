#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

# Restore firefox configuration
restore_firefox() {
   echo -e "\033[32mRestoring firefox user preference...\033[0m"
   # You need to open firefox first to create the folder
   read -p "Please open firefox first. Press [Enter] key to continue..."
   find /home/ditrobotics/snap/firefox/common/.mozilla/firefox/ -type d -name "*.default" -exec cp -r /home/ditrobotics/DIT-Scripts/.mozilla/firefox/dit_config.default/* {} \;
}

# Restore desktop configuration
restore_desktop() {
   echo -e "\033[32mRestoring desktop user preference...\033[0m"
   
   cp -r /home/ditrobotics/DIT-Scripts/desktop/* /home/ditrobotics/Desktop/
   
   if [ ! -d /home/share/scripts/ ]; then
      mkdir -p /home/share/scripts/
   fi
   cp -r /home/ditrobotics/DIT-Scripts/share/scripts/* /home/share/scripts/
   
   if [ ! -d /home/share/data/ ]; then
      mkdir -p /home/share/data/
   fi
   cp -r /home/ditrobotics/DIT-Scripts/share/data/* /home/share/data/
}

# Restore plymouth theme configuration
restore_plymouth() {
   echo -e "\033[32mRestoring plymouth theme preference...\033[0m"
   cp -r /home/ditrobotics/DIT-Scripts/system/plymouth-themes/abstract_ring_alt /usr/share/plymouth/themes/
   update-alternatives --install /usr/share/plymouth/themes/default.plymouth default.plymouth /usr/share/plymouth/themes/abstract_ring_alt/abstract_ring_alt.plymouth 100
   echo -e "\033[32mSelect the number for installed theme...\033[0m"
   update-alternatives --config default.plymouth
   update-initramfs -u
}

if [ "$1" == "all" ]; then
   restore_firefox
   restore_desktop
   restore_plymouth
else
   case "$1" in
      firefox)
         restore_firefox
         ;;
      desktop)
         restore_desktop
         ;;
      plymouth)
         restore_plymouth
         ;;
      *)
         echo "Usage: $0 {all|firefox|desktop|plymouth}"
         exit 1
         ;;
   esac
fi
